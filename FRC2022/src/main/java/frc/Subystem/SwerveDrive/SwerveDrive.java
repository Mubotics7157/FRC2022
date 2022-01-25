package frc.Subystem.SwerveDrive;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auto.PathTrigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.util.Threading.Threaded;
public class SwerveDrive extends Threaded{

    SwerveModules frontLeft = new SwerveModules(8, 1, 0);
    SwerveModules backLeft = new SwerveModules(2,3,4);
    SwerveModules frontRight = new SwerveModules(4, 5, 8);
    SwerveModules backRight = new SwerveModules(6, 7, 12);
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

    PIDController fwdController;
    PIDController strController;
    ProfiledPIDController rotController;

    Timer autoTimer = new Timer();
    //HolonomicDriveController autoController = new HolonomicDriveController(requireNonNullParam(fwdController,"y Controller","SwerveDrive"), requireNonNullParam(strController, "x Controller", "SwerveDrive"), requireNonNullParam(rotController, "theta Controller", "SwerveDrive"));
    HolonomicDriveController controller;

    Trajectory currTrajectory;
    private ArrayList<PathTrigger> triggers = new ArrayList<>();
    Rotation2d wantedHeading;

    private static SwerveDrive instance;

    SwerveState swerveState = SwerveState.FIELD_ORIENTED;

    public static SwerveDrive getInstance(){
        if(instance==null)
            instance = new SwerveDrive();
        return instance;
    }

    public enum SwerveState{
        REGULAR,
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
            case REGULAR:
                SmartDashboard.putString("Swerve State", "Driver Oriented");
                break;
            case FIELD_ORIENTED:
                SmartDashboard.putString("Swerve State", "Field Oriented");
                //updateFieldOriented(Robot.operator.getLeftX(), Robot.operator.getLeftY(), Robot.operator.getRightX());
                updateWPIFieldOriented(Robot.operator.getLeftY(), Robot.operator.getLeftX(), Robot.operator.getRightX());
                break;
            case VISION:
                SmartDashboard.putString("Swerve State", "Vision Tracking");
                break;
            case CARGO:
                SmartDashboard.putString("Swerve State", "Cargo Tracking");
                break;
            case AUTO:
                updateAuto();
                SmartDashboard.putString("Swerve State", "Auto");
                break;
            case DONE:
                SmartDashboard.putString("Swerve State", "Done");
                break;
        }

    if(Robot.isSimulation())
        updateSim();
    }

    private void updateFieldOriented(double fwd, double str, double rot){
        helper.updateTranslationalVectors(fwd, str, navX.getAngle(), rot);
        speeds = helper.calculateWheelSignals();
        angles = helper.calculateAzimuthAngles();
        frontLeft.set(speeds[0], angles[0]);
        frontRight.set(speeds[1], angles[1]);
        backLeft.set(speeds[2], angles[2]);
        backRight.set(speeds[3], angles[3]);
    }

    private void updateWPIFieldOriented(double fwd, double str, double rot){
        var states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(str*DriveConstants.MAX_SPEED_TELE, fwd*DriveConstants.MAX_SPEED_TELE, rot*DriveConstants.kMaxModuleAngularSpeedRadiansPerSecond, getDriveHeading()));
        DriveConstants.SWERVE_KINEMATICS.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED_TELE);
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);

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
        while (!triggers.isEmpty()) {
			if (triggers.get(0).getPercentage() <= getPathPercentage()) {
        triggers.remove(0).playTrigger();
			} else {
				break;
			}
		}
   
      Trajectory.State desiredPose = currTrajectory.sample(currentTime);
      ChassisSpeeds speeds = controller.calculate(SwerveTracker.getInstance().getOdometry(), desiredPose, Rotation2d.fromDegrees(0));
      SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
      SmartDashboard.putNumber("desired poseX", desiredPose.poseMeters.getX());
      SmartDashboard.putNumber("desired poseY", desiredPose.poseMeters.getY());
      SmartDashboard.putNumber("desired angle", desiredPose.poseMeters.getRotation().getDegrees());
      boolean isFinished= autoTimer.advanceIfElapsed(currTrajectory.getTotalTimeSeconds());
      if(isFinished){
          
      }
      frontLeft.setState(moduleStates[0]);
      frontRight.setState(moduleStates[1]);
      backLeft.setState(moduleStates[2]);
      backRight.setState(moduleStates[3]);
      
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

    public synchronized void setAutoPath(Trajectory desiredTrajectory){
        autoTimer.reset();
        autoTimer.start();;
        currTrajectory = desiredTrajectory;
        swerveState = SwerveState.AUTO;
        updateAuto();
    }

    public synchronized void setAutoPath(Trajectory desiredTrajectory,ArrayList<PathTrigger> triggers){
        autoTimer.reset();
        autoTimer.start();;
        currTrajectory = desiredTrajectory;
        this.triggers = triggers;
        swerveState = SwerveState.AUTO;
        updateAuto();
    }
}
/*1150
1480


8in 7 in compression*/