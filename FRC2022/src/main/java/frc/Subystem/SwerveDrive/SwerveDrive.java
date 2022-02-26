package frc.Subystem.SwerveDrive;


import java.util.ArrayList;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

    SwerveModules backRight = new SwerveModules(1, 2, 3,-80,.3);
    SwerveModules frontRight = new SwerveModules(7,8,9,-111,.3);
    SwerveModules backLeft = new SwerveModules(4, 5, 6,-6,.22);
    SwerveModules frontLeft = new SwerveModules(10, 11, 12,0,.24);
    SwerveModules[] modules = {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    };

    private WPI_Pigeon2 pigeon = new WPI_Pigeon2(30);


    PIDController fwdController = new PIDController(.1, 0, 0);
    PIDController strController = new PIDController(.1, 0, 0);
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(6.28,3.14);
    ProfiledPIDController rotController = new ProfiledPIDController(.1, 0, 0,rotProfile);
    //rotControler.enableContinuousInput(-Math.PI,Math.PI);
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
        turnPID = new SynchronousPID(.1, 0, 0);
        distancePID = new SynchronousPID(0, 0, 0);
        headingPID = new SynchronousPID(.004, 0, 0);

        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPoseYaw = 90;
        pigeon.reset();
        pigeon.calibrate();

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
        double fwd = Robot.operator.getLeftY();
        double str = Robot.operator.getLeftX();
        double rot = Robot.operator.getRightX();
        if(Math.abs(fwd) <= .2)
            fwd = 0;
        if(Math.abs(str) <= .2)
            str = 0;
        if(Math.abs(rot) <= .2)
            rot = 0;
        
        driveWPIFieldOriented(-.7, str, .3);
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
      PathPlannerState state = (PathPlannerState)currTrajectory.sample(currentTime);
      ChassisSpeeds speeds = controller.calculate(SwerveTracker.getInstance().getOdometry(), desiredPose, state.holonomicRotation);
      SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
      SmartDashboard.putNumber("desired poseX", desiredPose.poseMeters.getX());
      SmartDashboard.putNumber("desired poseY", desiredPose.poseMeters.getY());
      SmartDashboard.putNumber("desired angle", desiredPose.poseMeters.getRotation().getDegrees());
      boolean isFinished= autoTimer.advanceIfElapsed(currTrajectory.getTotalTimeSeconds());
      if(isFinished){
          swerveState = SwerveState.DONE;
      }
      frontLeft.setState(moduleStates[0]);
      frontRight.setState(moduleStates[1]);
      backLeft.setState(moduleStates[2]);
      backRight.setState(moduleStates[3]);
    }


    private void updateVisionTracking(){
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getDegrees();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        driveRobotOriented(Robot.operator.getLeftY(), Robot.operator.getLeftY(), deltaSpeed);
    }


    private void updateAlign(){
       if(VisionManager.getInstance().foundTargets())
        {
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getDegrees();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        if(error<3 &&deltaSpeed<.01)
            setVisionTracking();
        else
        driveRobotOriented(0, 0, deltaSpeed);
        }

        else
            driveRobotOriented(0, 0, -.7);

    }

    private void updateRobotOriented(){
        double fwd = Robot.operator.getLeftY();
        double str = Robot.operator.getLeftX();
        double rot = Robot.operator.getRightX();
        double correction = 0;

        if(Math.abs(fwd) <= .2)
            fwd = 0;
        if(Math.abs(str) <= .4)
            str = 0;
        if(Math.abs(rot) <= .2)
            rot = 0;
        
        if(Math.abs(fwd)>0 && rot==0){
            double error = Rotation2d.fromDegrees(0).rotateBy(getDriveHeading()).unaryMinus().getDegrees();
            correction = headingPID.update(error);
        }
        driveRobotOriented(fwd, str, rot+correction);

    }

    private void updateCargoLock(){
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getDegrees();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        driveRobotOriented(Robot.operator.getLeftY(), Robot.operator.getLeftX(), Robot.operator.getRawAxis(2));
    }
    private void setModuleStates(SwerveModuleState[] states){
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    private void driveWPIFieldOriented(double fwd, double str, double rot){
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD, getDriveHeading()));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 2);
        setModuleStates(states);
    }

    private void driveRobotOriented(double fwd, double str, double rot){
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*(4*Math.PI)));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
        SmartDashboard.putNumber("actual angle", backLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("module velocity", backLeft.getState().speedMetersPerSecond);
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
}