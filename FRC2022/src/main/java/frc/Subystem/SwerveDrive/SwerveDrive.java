package frc.Subystem.SwerveDrive;


import java.util.ArrayList;

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
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.VisionManager;
import frc.auto.PathTrigger;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.util.Threading.Threaded;
public class SwerveDrive extends Threaded{

    SwerveModules backRight = new SwerveModules(25, 2, 3,DriveConstants.BR_OFFSET);
    SwerveModules frontRight = new SwerveModules(7,8,9,DriveConstants.FR_OFFSET);
    SwerveModules backLeft = new SwerveModules(4, 5, 6,DriveConstants.BL_OFFSET);
    SwerveModules frontLeft = new SwerveModules(10, 11, 12,DriveConstants.FL_OFFSET);
    SwerveModules[] modules = { 
        frontLeft,
        frontRight,
        backLeft,
        backRight
    };

    private AHRS gyro = new AHRS(Port.kMXP);


    PIDController fwdController = new PIDController(DriveConstants.FWD_kP, 0, 0);
    PIDController strController = new PIDController(DriveConstants.STR_kP, 0, 0);
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(4,4);
    TrapezoidProfile.Constraints visionRotProfile = new TrapezoidProfile.Constraints(3*Math.PI,3*Math.PI);
    ProfiledPIDController visionRotController = new ProfiledPIDController(DriveConstants.THETA_kP, 0, DriveConstants.THETA_kD,visionRotProfile); //.2
    ProfiledPIDController rotController = new ProfiledPIDController(DriveConstants.THETA_kP, 0, 0,rotProfile); //.2
    Rotation2d desiredAutoHeading;

    HolonomicDriveController controller = new HolonomicDriveController(strController, fwdController, rotController);

    
    Timer autoTimer;

    Trajectory currTrajectory;
    private ArrayList<PathTrigger> triggers;

    private static SwerveDrive instance;

    private SwerveState swerveState = SwerveState.ROBOT_ORIENTED;
    boolean rotateOnCenter = true;

    public SwerveDrive(){
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        visionRotController.enableContinuousInput(-Math.PI, Math.PI);

        //gyro.reset();
        //gyro.zeroYaw();
        //gyro.calibrate();
        autoTimer = new Timer();
        triggers = new ArrayList<>();
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
                SmartDashboard.putString("drive state", "robot oriented");
                break;
            case FIELD_ORIENTED:
                updateManual(true);
                SmartDashboard.putString("drive state", "field oriented");
                break;
            case ALIGN:
                updateAlign();
                SmartDashboard.putString("drive state", "aligning");
                break;
            case AUTO:
                updateAuto();
                SmartDashboard.putString("drive state", "auto running");
                break;
            case DONE:
                stopMotors();
                SmartDashboard.putString("drive state", "done");
                break;
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
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, str*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD_TELEOP, getDriveHeading().unaryMinus());
        else
            speeds = new ChassisSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, str*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD_TELEOP);

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
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, str*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD_TELEOP, getDriveHeading().unaryMinus());
        else
            speeds = new ChassisSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, str*DriveConstants.MAX_TANGENTIAL_VELOCITY_TELEOP, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD_TELEOP);
        
        driveFromChassis(speeds);
    }

    private void updateAlign(){
       if(VisionManager.getInstance().hasVisionTarget())
        {
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).getRadians();
        if(Math.abs(error)<Units.degreesToRadians(3))
            error = 0;
        double deltaSpeed = visionRotController.calculate(error);
        updateManual(true,deltaSpeed);
        }
       else{
            setFieldOriented();
        }
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
    }

    public synchronized void calibrateGyro(){
        gyro.calibrate();
    }


}