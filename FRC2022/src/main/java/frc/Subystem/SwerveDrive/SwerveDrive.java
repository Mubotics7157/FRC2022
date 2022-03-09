package frc.Subystem.SwerveDrive;


import java.util.ArrayList;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.VisionManager;
import frc.auto.PathTrigger;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.util.Threading.Threaded;
public class SwerveDrive extends Threaded{

    SwerveModules backRight = new SwerveModules(25, 2, 3,-80.439453125);
    SwerveModules frontRight = new SwerveModules(7,8,9,-111.263671875);
    SwerveModules backLeft = new SwerveModules(4, 5, 6,-6-2.263671875);
    SwerveModules frontLeft = new SwerveModules(10, 11, 12,2.724609375);
    SwerveModules[] modules = { 
        frontLeft,
        frontRight,
        backLeft,
        backRight
    };

    private AHRS gyro = new AHRS(Port.kMXP);


    PIDController fwdController = new PIDController(2, 0, 0);
    PIDController strController = new PIDController(2, 0, 0);
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(4,4);
    TrapezoidProfile.Constraints visionRotProfile = new TrapezoidProfile.Constraints(3*Math.PI,3*Math.PI);
    ProfiledPIDController visionRotController = new ProfiledPIDController(.22, 0, .02,visionRotProfile); //.2
    ProfiledPIDController rotController = new ProfiledPIDController(.22, 0, 0,rotProfile); //.2
    Rotation2d desiredAutoHeading;

    HolonomicDriveController controller = new HolonomicDriveController(strController, fwdController, rotController);

    
    Timer autoTimer = new Timer();

    Trajectory currTrajectory;
    private ArrayList<PathTrigger> triggers = new ArrayList<>();

    private static SwerveDrive instance;

    private SwerveState swerveState = SwerveState.ROBOT_ORIENTED;

    public SwerveDrive(){
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        visionRotController.enableContinuousInput(-Math.PI, Math.PI);

        gyro.reset();
        gyro.zeroYaw();
        gyro.calibrate();
        rotController.setTolerance(Units.degreesToRadians(10));
        visionRotController.setTolerance(Units.degreesToRadians(10));
        controller.setTolerance(new Pose2d(.5,.5,Rotation2d.fromDegrees(10)));

    }

    public static SwerveDrive getInstance(){
        if(instance==null)
            instance = new SwerveDrive();
        return instance;
    }

    public enum SwerveState{
        ROBOT_ORIENTED,
        FIELD_ORIENTED,
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
                updateManual(false);
                SmartDashboard.putString("Swerve State", "Driver Oriented");
                break;
            case FIELD_ORIENTED:
                SmartDashboard.putString("Swerve State", "Field Oriented");
                updateManual(true);
                break;
            /*
            case ALIGN:
                updateAlign();
                break;
                */
                /*
            case AUTO:
                updateAuto();
                SmartDashboard.putString("Swerve State", "Auto");
                break;
            case DONE:
                stopMotors();
                SmartDashboard.putString("Swerve State", "Done");
                break;
                */
        }
    }
        private void updateManual(boolean fieldOriented){
        ChassisSpeeds speeds;
        double fwd = Robot.driver.getRawAxis(1);
        double str = Robot.driver.getRawAxis(0);
        double rot = Robot.driver.getRawAxis(4);

        if(Math.abs(fwd) <= .1)
            fwd = 0;
        if(Math.abs(str) <= .1)
            str = 0;
        if(Math.abs(rot) <= .1)
            rot = 0;

        if(fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD, getDriveHeading().unaryMinus());
        else
            speeds = new ChassisSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD);

        driveFromChassis(speeds);
    }

    private void updateManual(boolean fieldOriented, double rotModifier){
        ChassisSpeeds speeds;
        double fwd = Robot.driver.getRawAxis(1);
        double str = Robot.driver.getRawAxis(0);
        double rot = rotModifier;

        if(Math.abs(fwd) <= .05)
            fwd = 0;
        if(Math.abs(str) <= .05)
            str = 0;
        if(fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD, getDriveHeading());
        else
            speeds = new ChassisSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD);
        
        driveFromChassis(speeds);
    }

    private void updateAlign(){
       //if(VisionManager.getInstance().hasVisionTarget())
       // {
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).getRadians();
        if(Math.abs(error)<Units.degreesToRadians(3))
            error = 0;
        double deltaSpeed = visionRotController.calculate(error);
        updateManual(true,deltaSpeed);
        if(error==0)
            setFieldOriented();
       // }
        //else{
          //  setFieldOriented();
       // }
    }

    private void updateAuto(){
        double currentTime = autoTimer.get();
        //SmartDashboard.putBoolean("finished", isFinished());
        //SmartDashboard.putNumber("trajectory time", currTrajectory.getTotalTimeSeconds());
        //SmartDashboard.putNumber("elapsed time", autoTimer.get());
        while (!triggers.isEmpty()) {
			if (triggers.get(0).getPercentage() <= getPathPercentage()) {
        triggers.remove(0).playTrigger();
			} else {
				break;
			}
		}
      Trajectory.State desiredPose = currTrajectory.sample(currentTime);
    //   SmartDashboard.putNumber("desired rotation", desiredPose.poseMeters.getRotation().getDegrees());
    //   SmartDashboard.putNumber("desired posex", desiredPose.poseMeters.getX());
    //   SmartDashboard.putNumber("desired posey", desiredPose.poseMeters.getY());
      ChassisSpeeds speeds = controller.calculate(SwerveTracker.getInstance().getOdometry(), desiredPose,Rotation2d.fromDegrees(0));
      driveFromChassis(speeds);
    //   SmartDashboard.putNumber("error poseX", desiredPose.poseMeters.getX()-SwerveTracker.getInstance().getOdometry().getX());
    //   SmartDashboard.putNumber("error poseY", desiredPose.poseMeters.getY()-SwerveTracker.getInstance().getOdometry().getY());
    //   SmartDashboard.putNumber("error poseR", desiredPose.poseMeters.getRotation().getDegrees()-SwerveTracker.getInstance().getOdometry().getRotation().getDegrees());
      boolean isFinished= autoTimer.hasElapsed(currTrajectory.getTotalTimeSeconds());
    //   SmartDashboard.putBoolean("has elapsed", isFinished);
    //   SmartDashboard.putBoolean("at reference", controller.atReference());
      if(isFinished){
         swerveState = SwerveState.DONE;
       }
    }

    private void driveFromChassis(ChassisSpeeds speeds) {
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
    }   

    private void driveFromChassis(ChassisSpeeds speeds, Translation2d centerOfRotation) {
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
    }
    private void setModuleStates(SwerveModuleState[] states){
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    public synchronized SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = {frontLeft.getState(),frontRight.getState(),backLeft.getState(),backRight.getState()};
        return states;
    }

    public synchronized Rotation2d getDriveHeading(){
        return Rotation2d.fromDegrees(gyro.getYaw());
    }


    private double getPathPercentage(){
        return autoTimer.get()/currTrajectory.getTotalTimeSeconds();
    }
    public synchronized boolean isFinished(){
        return swerveState == SwerveState.DONE;
    }

    public synchronized void zeroYaw(){
        gyro.reset();
       SwerveTracker.getInstance().setOdometry(new Pose2d(SwerveTracker.getInstance().getOdometry().getTranslation(), Rotation2d.fromDegrees(0)));
    }

    private void stopMotors(){
        frontLeft.overrideMotors();
        frontRight.overrideMotors();
        backLeft.overrideMotors();
        backRight.overrideMotors();
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

    public synchronized void increaseD(){
        double num = SmartDashboard.getNumber("turning d", 0);
        frontRight.updateD(num);
        backLeft.updateD(num);
        backRight.updateD(num);
        frontLeft.updateD(num);
    }
    public synchronized void increaseP(){
        double num = SmartDashboard.getNumber("turning p", 0);
        frontRight.updateP(num);
        backLeft.updateP(num);
        backRight.updateP(num);
        frontLeft.updateP(num);
    }

    public synchronized void resetGyro(){
        gyro.reset();
        gyro.calibrate();
    }

}