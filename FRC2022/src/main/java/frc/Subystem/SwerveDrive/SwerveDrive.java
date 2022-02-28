package frc.Subystem.SwerveDrive;


import java.util.ArrayList;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.VisionManager;
import frc.auto.PathTrigger;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.util.SynchronousPID;
import frc.util.Threading.Threaded;
public class SwerveDrive extends Threaded{

    SwerveModules backRight = new SwerveModules(1, 2, 3,-80,.27);//.27);
    SwerveModules frontRight = new SwerveModules(7,8,9,-111,.27);//.27);
    SwerveModules backLeft = new SwerveModules(4, 5, 6,-6,.26);//26);
    SwerveModules frontLeft = new SwerveModules(10, 11, 12,0,.27);//.27); //.55
    SwerveModules[] modules = { 
        frontLeft,
        frontRight,
        backLeft,
        backRight
    };

    private WPI_Pigeon2 pigeon = new WPI_Pigeon2(30);


    PIDController fwdController = new PIDController(1, 0, 0);
    PIDController strController = new PIDController(1, 0, 0);
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(3*Math.PI,3*Math.PI);
    ProfiledPIDController rotController = new ProfiledPIDController(.2, 0, 0,rotProfile);
    SynchronousPID turnPID;
    SynchronousPID distancePID;
    SynchronousPID headingPID;

    HolonomicDriveController controller = new HolonomicDriveController(strController, fwdController, rotController);

    Timer autoTimer = new Timer();

    Trajectory currTrajectory;
    PathPlannerTrajectory currentPlannerTrajectory;
    private ArrayList<PathTrigger> triggers = new ArrayList<>();
    Rotation2d wantedHeading;

    private static SwerveDrive instance;

    private SwerveState swerveState = SwerveState.ROBOT_ORIENTED;

    public SwerveDrive(){
        turnPID = new SynchronousPID(-.00125, 0, 0);
        distancePID = new SynchronousPID(0, 0, 0);

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPoseYaw = 90;
        pigeon.reset();
        pigeon.calibrate();
        turnPID.setTolerance(1);

    }

    public static SwerveDrive getInstance(){
        if(instance==null)
            instance = new SwerveDrive();
        return instance;
    }

    public enum SwerveState{
        ROBOT_ORIENTED,
        FIELD_ORIENTED,
        VISION,
        CARGO,
        AUTO,
        ALIGN,
        DONE
    }

    @Override
    public void update() {
        SwerveState snapSwerveState;
        synchronized(this){
            snapSwerveState = swerveState;
        }
        switch(snapSwerveState){
            case ROBOT_ORIENTED:
                updateRobotOriented();
                SmartDashboard.putString("Swerve State", "Driver Oriented");
                break;
            case FIELD_ORIENTED:
                SmartDashboard.putString("Swerve State", "Field Oriented");
                updateFieldOriented();
                break;
            case VISION:
                SmartDashboard.putString("Swerve State", "Vision Tracking");
                updateVisionTracking();
                break;
            case CARGO:
                updateCargoLock();
                SmartDashboard.putString("Swerve State", "Cargo Tracking");
                break;
            case ALIGN:
                SmartDashboard.putString("Swerve State", "Align");
                updateAlign();
                break;
            case AUTO:
                updateAuto();
                SmartDashboard.putString("Swerve State", "Auto");
                break;
            case DONE:
                SmartDashboard.putString("Swerve State", "Done");
                break;
        }
    }

    private void updateFieldOriented(){
        double fwd = Robot.driver.getLeftY();
        double str = Robot.driver.getLeftX();
        double rot = Robot.driver.getRightX();
        if(Math.abs(fwd) <= .05)
            fwd = 0;
        if(Math.abs(str) <= .05)
            str = 0;
        if(Math.abs(rot) <= .05)
            rot = 0;
        
        driveWPIFieldOriented(fwd, str, rot);
        SmartDashboard.putNumber("rotation", rot);
        SmartDashboard.putNumber("forward", fwd);
        SmartDashboard.putNumber("strafe", str);
    }

    private void updateAuto(){
    
        double currentTime = autoTimer.get();
        SmartDashboard.putBoolean("finished", isFinished());
        //SmartDashboard.putNumber("trajectory time", currTrajectory.getTotalTimeSeconds());
        SmartDashboard.putNumber("elapsed time", autoTimer.get());
        while (!triggers.isEmpty()) {
			if (triggers.get(0).getPercentage() <= getPathPercentage()) {
        triggers.remove(0).playTrigger();
			} else {
				break;
			}
		}
      Trajectory.State desiredPose = currTrajectory.sample(currentTime);
      //PathPlannerState state = (PathPlannerState)currTrajectory.sample(currentTime);
      ChassisSpeeds speeds = controller.calculate(SwerveTracker.getInstance().getOdometry(), desiredPose,Rotation2d.fromDegrees(0));//, state.holonomicRotation);
      SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
      SmartDashboard.putNumber("desired poseX", desiredPose.poseMeters.getX());
      SmartDashboard.putNumber("desired poseY", desiredPose.poseMeters.getY());
      SmartDashboard.putNumber("desired angle", desiredPose.poseMeters.getRotation().getDegrees());

      SmartDashboard.putNumber("error poseX", desiredPose.poseMeters.getX()-SwerveTracker.getInstance().getOdometry().getX());
      SmartDashboard.putNumber("error poseY", desiredPose.poseMeters.getY()-SwerveTracker.getInstance().getOdometry().getY());
      SmartDashboard.putNumber("error poseR", desiredPose.poseMeters.getRotation().getDegrees()-SwerveTracker.getInstance().getOdometry().getRotation().getDegrees());
      boolean isFinished= autoTimer.advanceIfElapsed(currTrajectory.getTotalTimeSeconds());
      if(isFinished){
          swerveState = SwerveState.DONE;
      }
      else{
      frontLeft.setState(moduleStates[0]);
      frontRight.setState(moduleStates[1]);
      backLeft.setState(moduleStates[2]);
      backRight.setState(moduleStates[3]);
    }
    }


    private void updateVisionTracking(){
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).unaryMinus().getDegrees();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        driveRobotOriented(Robot.driver.getLeftY(), Robot.driver.getLeftY(), deltaSpeed);
    }


    private void updateAlign(){
       if(VisionManager.getInstance().hasVisionTarget())
        {
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).unaryMinus().getDegrees();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        //if(error<3 &&deltaSpeed<.01)
          //  setVisionTracking();
        //else
        driveRobotOriented(Robot.driver.getLeftX() , Robot.driver.getLeftY(), deltaSpeed);
        }
        //else
        //setFieldOriented();
    }

    private void updateRobotOriented(){
        double fwd = Robot.driver.getLeftY();
        double str = Robot.driver.getLeftX();
        double rot = Robot.driver.getRightX();
        double correction = 0;

        if(Math.abs(fwd) <= .05)
            fwd = 0;
        if(Math.abs(str) <= .05)
            str = 0;
        if(Math.abs(rot) <= .05)
            rot = 0;
        
        if(Math.abs(fwd)>0 && rot==0){
            double error = Rotation2d.fromDegrees(0).rotateBy(getDriveHeading()).unaryMinus().getDegrees();
            correction = headingPID.update(error);
        }
        driveRobotOriented(fwd, str, rot+correction);

    }

    private void updateCargoLock(){
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getCargoYawRotation2d()).unaryMinus().getDegrees();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        driveRobotOriented(Robot.driver.getLeftY(), Robot.driver.getLeftX(), Robot.driver.getRawAxis(2));
    }
    private void setModuleStates(SwerveModuleState[] states){
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    private void driveWPIFieldOriented(double fwd, double str, double rot){
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD, getDriveHeading()));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
        SmartDashboard.putNumber("left front angle", frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("left back angle", backLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber(" right front angle", frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber(" right back angle", backRight.getState().angle.getDegrees());
    }

    private void driveRobotOriented(double fwd, double str, double rot){
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*(4*Math.PI)));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
        SmartDashboard.putNumber("left front angle", frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("left back angle", backLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber(" right front angle", frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber(" right back angle", backRight.getState().angle.getDegrees());
    }
    public synchronized SwerveModules[] getModules(){
        return modules;
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = {frontLeft.getState(),frontRight.getState(),backLeft.getState(),backRight.getState()};
        return states;
    }

    public synchronized Rotation2d getDriveHeading(){
        return pigeon.getRotation2d();
    }

    public synchronized double getGyroAngle(){
        return pigeon.getAngle();
    }

    public boolean isFinished(){
        return swerveState == SwerveState.DONE;
    }
    private double getPathPercentage(){
        return autoTimer.get()/currTrajectory.getTotalTimeSeconds();
    }

    public synchronized void setFieldOriented(){
        swerveState = SwerveState.FIELD_ORIENTED;
    }

    public synchronized void setTargetAlign(){
        swerveState = SwerveState.ALIGN;
    }

    public synchronized void setRobotOriented(){
        swerveState = SwerveState.ROBOT_ORIENTED;
    }
    public synchronized void setAutoPath(Trajectory desiredTrajectory){
        autoTimer.reset();
        autoTimer.start();
        currTrajectory = desiredTrajectory;
        swerveState = SwerveState.AUTO;
        updateAuto();
    }


    public void setAutoPath(Trajectory desiredTrajectory,ArrayList<PathTrigger> triggers){
        autoTimer.reset();
        autoTimer.start();
        currTrajectory = desiredTrajectory;
        this.triggers = triggers;
        synchronized(this){
        swerveState = SwerveState.AUTO;
        }
        updateAuto();
    }

    public synchronized void setCargoLock(){
        swerveState = SwerveState.CARGO;
    }

    public synchronized void setVisionTracking(){
        swerveState = SwerveState.VISION;
    }

    public synchronized void increaseD(){
        //
        frontRight.updateD(SmartDashboard.getNumber("turning d ", 0));
        backLeft.updateD(SmartDashboard.getNumber("turning d ", 0));
        backRight.updateD(SmartDashboard.getNumber("turning d ", 0));
        frontLeft.updateD(SmartDashboard.getNumber("turning d ", 0));
    }
    public synchronized void increaseP(){
        //
        frontRight.updateP(SmartDashboard.getNumber("turning p ", 0));
        backLeft.updateP(SmartDashboard.getNumber("turning p ", 0));
        backRight.updateP(SmartDashboard.getNumber("turning p ", 0));
        frontLeft.updateP(SmartDashboard.getNumber("turning p ", 0));
    }

    public synchronized void zeroYaw(){
        pigeon.reset();
        SwerveTracker.getInstance().setOdometry(new Pose2d(SwerveTracker.getInstance().getOdometry().getTranslation(), Rotation2d.fromDegrees(0)));
    }

    public synchronized void increaseLLP(){
        turnPID.setP(SmartDashboard.getNumber("LL P", 0));
    }
}