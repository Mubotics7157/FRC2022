package frc.Subystem.SwerveDrive;


import java.util.ArrayList;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.VisionManager;
import frc.auto.PathTrigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.util.SynchronousPID;
import frc.util.Threading.Threaded;
public class SwerveDrive extends Threaded{

    SwerveModules frontLeft = new SwerveModules(8, 1, 0);
    SwerveModules backLeft = new SwerveModules(2,3,4);
    SwerveModules frontRight = new SwerveModules(4, 5, 8);
    SwerveModules backRight = new SwerveModules(6, 7, 13);
    SwerveModules[] modules = {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    };

    private AnalogGyro navX = new AnalogGyro(0);
    AnalogGyroSim navXSim = new AnalogGyroSim(navX);

    double [] angles= new double[4];
    double[]speeds = new double[4];
    ModuleHelper helper = new ModuleHelper();
    double yawVal = 0;

    PIDController fwdController = new PIDController(1, 0, 0);
    PIDController strController = new PIDController(1, 0, 0);
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(3*Math.PI,3*Math.PI);
    ProfiledPIDController rotController = new ProfiledPIDController(2, 0, 0,rotProfile);
    //rotControler.enableContinuousInput(-Math.PI,Math.PI);
    SynchronousPID turnPID;

    HolonomicDriveController controller = new HolonomicDriveController(strController, fwdController, rotController);

    Timer autoTimer = new Timer();

    Trajectory currTrajectory;
    PathPlannerTrajectory currentPlannerTrajectory;
    private ArrayList<PathTrigger> triggers = new ArrayList<>();
    Rotation2d wantedHeading;

    private static SwerveDrive instance;

    private SwerveState swerveState = SwerveState.ROBOT_ORIENTED;

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
            case AUTO:
                updateAuto();
                SmartDashboard.putString("Swerve State", "Auto");
                break;
            case DONE:
                updateDone();
                SmartDashboard.putString("Swerve State", "Done");
                break;
        }

    if(Robot.isSimulation())
        updateSim();
    }


    private void updateFieldOriented(){
        driveWPIFieldOriented(Robot.operator.getLeftY(), Robot.operator.getLeftX(), Robot.operator.getRawAxis(2));
    }


    public void updateSim(){
        frontLeft.updateSim();
        frontRight.updateSim();
        backLeft.updateSim();
        backRight.updateSim();
        SwerveModuleState[] moduleStates = getModuleStates();

        var chassisSpeed = Constants.DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

        yawVal += chassisRotationSpeed * 0.02;
        navXSim.setAngle(-Units.radiansToDegrees(yawVal));
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
        driveWPIFieldOriented(Robot.operator.getLeftY(), Robot.operator.getLeftX(), Robot.operator.getRightX()+deltaSpeed);
    }

    private void updateTargetTracking(){
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getDegrees();
        double errorRad = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getRadians();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        driveWPIFieldOriented(Robot.operator.getLeftY(), Robot.operator.getLeftX(), Robot.operator.getRightX()+deltaSpeed);
        driveRobotOriented(0, 0, deltaSpeed);
    }

    private void turnToTarget(){
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getDegrees();
        double errorRad = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getRadians();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(errorRad);
        SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
        driveRobotOriented(0, 0, deltaSpeed);
    }

    private void updateRobotOriented(){
        driveRobotOriented(-Robot.operator.getLeftX(), -Robot.operator.getLeftY(), Robot.operator.getRightX());

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

    private void updateDone(){
        driveWPIFieldOriented(0, 0, 0);
    }

    private void driveWPIFieldOriented(double fwd, double str, double rot){
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(str*DriveConstants.MAX_TANGENTIAL_VELOCITY, fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY_RAD, getDriveHeading()));
        DriveConstants.SWERVE_KINEMATICS.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
        SmartDashboard.putNumber("desired angle", states[0].angle.getDegrees());
    }

    private void driveRobotOriented(double fwd, double str, double rot){
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(str*DriveConstants.MAX_TANGENTIAL_VELOCITY, fwd*DriveConstants.MAX_TANGENTIAL_VELOCITY, rot*DriveConstants.MAX_ANGULAR_VELOCITY));
        DriveConstants.SWERVE_KINEMATICS.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
        SmartDashboard.putNumber("desired angle", states[0].angle.getDegrees());
        SmartDashboard.putNumber("actual angle", frontLeft.getState().angle.getDegrees());
      SmartDashboard.putNumber("velocity setpoint", states[0].speedMetersPerSecond);
      SmartDashboard.putNumber("actual velocity", frontLeft.getState().speedMetersPerSecond);
        
    }

    public synchronized SwerveModules[] getModules(){
        return modules;
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = {frontLeft.getState(),frontRight.getState(),backLeft.getState(),backRight.getState()};
        return states;
    }

    public synchronized Rotation2d getDriveHeading(){
        return navX.getRotation2d();
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