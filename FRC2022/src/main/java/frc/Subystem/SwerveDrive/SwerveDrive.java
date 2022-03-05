package frc.Subystem.SwerveDrive;


import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

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

    SwerveModules backRight = new SwerveModules(25, 2, 3,-80);
    SwerveModules frontRight = new SwerveModules(7,8,9,-111);
    SwerveModules backLeft = new SwerveModules(4, 5, 6,-6);
    SwerveModules frontLeft = new SwerveModules(10, 11, 12,0);
    SwerveModules[] modules = { 
        frontLeft,
        frontRight,
        backLeft,
        backRight
    };

    private AHRS gyro = new AHRS(Port.kMXP);


    PIDController fwdController = new PIDController(.2, 0, 0);
    PIDController strController = new PIDController(.2, 0, 0);
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(3*Math.PI,3*Math.PI);
    ProfiledPIDController rotController = new ProfiledPIDController(0.22, 0, 0,rotProfile); 

    HolonomicDriveController controller = new HolonomicDriveController(strController, fwdController, rotController);

    
    Timer autoTimer = new Timer();

    Trajectory currTrajectory;
    private ArrayList<PathTrigger> triggers = new ArrayList<>();

    private static SwerveDrive instance;

    private SwerveState swerveState = SwerveState.ROBOT_ORIENTED;

    public SwerveDrive(){
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        gyro.reset();
        gyro.zeroYaw();
        gyro.calibrate();
        rotController.setTolerance(Units.degreesToRadians(10));

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
            case ALIGN:
                SmartDashboard.putString("Swerve State", "Align");
                updateAlign();
                break;
            case AUTO:
                updateAuto();
                SmartDashboard.putString("Swerve State", "Auto");
                break;
            case DONE:
                stopMotors();
                SmartDashboard.putString("Swerve State", "Done");
                break;
        }
    }
        private void updateManual(boolean fieldOriented){
        ChassisSpeeds speeds;
        double fwd = Robot.driver.getRawAxis(1);
        double str = Robot.driver.getRawAxis(0);
        double rot = Robot.driver.getRawAxis(4);

        if(Math.abs(fwd) <= .05)
            fwd = 0;
        if(Math.abs(str) <= .05)
            str = 0;
        if(Math.abs(rot) <= .05)
            rot = 0;

        if(fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD, getDriveHeading());
        else
            speeds = new ChassisSpeeds(fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, str*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD);

        driveFromChassis(speeds);
        SmartDashboard.putNumber("left front angle", frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("left back angle", backLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("right front angle", frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("right back angle", backRight.getState().angle.getDegrees());

        SmartDashboard.putNumber("left front velocity", frontLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("left back velocity", backLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("right front velocity", frontRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("right back velocity", backRight.getState().speedMetersPerSecond);
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
       if(VisionManager.getInstance().hasVisionTarget())
        {
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).getRadians();
        if(Math.abs(error)<Units.degreesToRadians(3))
            error = 0;
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = rotController.calculate(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        updateManual(true,deltaSpeed);
        }
        else{
            setFieldOriented();
        }
    }

    private void updateAuto(){
        double currentTime = autoTimer.get();
        SmartDashboard.putBoolean("finished", isFinished());
        SmartDashboard.putNumber("trajectory time", currTrajectory.getTotalTimeSeconds());
        SmartDashboard.putNumber("elapsed time", autoTimer.get());
        while (!triggers.isEmpty()) {
			if (triggers.get(0).getPercentage() <= getPathPercentage()) {
        triggers.remove(0).playTrigger();
			} else {
				break;
			}
		}
      Trajectory.State desiredPose = currTrajectory.sample(currentTime);
      SmartDashboard.putNumber("desired rotation", desiredPose.poseMeters.getRotation().getDegrees());
      SmartDashboard.putNumber("desired posex", desiredPose.poseMeters.getX());
      SmartDashboard.putNumber("desired posey", desiredPose.poseMeters.getY());
      ChassisSpeeds speeds = controller.calculate(SwerveTracker.getInstance().getOdometry(), desiredPose,Rotation2d.fromDegrees(0));
      driveFromChassis(speeds);
      SmartDashboard.putNumber("error poseX", desiredPose.poseMeters.getX()-SwerveTracker.getInstance().getOdometry().getX());
      SmartDashboard.putNumber("error poseY", desiredPose.poseMeters.getY()-SwerveTracker.getInstance().getOdometry().getY());
      SmartDashboard.putNumber("error poseR", desiredPose.poseMeters.getRotation().getDegrees()-SwerveTracker.getInstance().getOdometry().getRotation().getDegrees());
      boolean isFinished= autoTimer.advanceIfElapsed(currTrajectory.getTotalTimeSeconds());
      if(isFinished && controller.atReference()){
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
        SmartDashboard.putNumber("gyro heading", gyro.getRotation2d().getDegrees());
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    public synchronized double getGyroAngle(){
        return Math.IEEEremainder(gyro.getYaw(), 360) *-1; 
    }

    private double getPathPercentage(){
        return autoTimer.get()/currTrajectory.getTotalTimeSeconds();
    }
    public boolean isFinished(){
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
        frontRight.updateD(SmartDashboard.getNumber("turning d ", 0));
        backLeft.updateD(SmartDashboard.getNumber("turning d ", 0));
        backRight.updateD(SmartDashboard.getNumber("turning d ", 0));
        frontLeft.updateD(SmartDashboard.getNumber("turning d ", 0));
    }
    public synchronized void increaseP(){
        frontRight.updateP(SmartDashboard.getNumber("turning p ", 0));
        backLeft.updateP(SmartDashboard.getNumber("turning p ", 0));
        backRight.updateP(SmartDashboard.getNumber("turning p ", 0));
        frontLeft.updateP(SmartDashboard.getNumber("turning p ", 0));
    }



}