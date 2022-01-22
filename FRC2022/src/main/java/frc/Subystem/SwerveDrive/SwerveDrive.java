package frc.Subystem.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
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
        CARGO
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
                updateFieldOriented(Robot.operator.getLeftX(), Robot.operator.getLeftY(), Robot.operator.getRightX());
                break;
            case VISION:
                SmartDashboard.putString("Swerve State", "Vision Tracking");
                break;
            case CARGO:
                SmartDashboard.putString("Swerve State", "Cargo Tracking");
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

    public synchronized void setFieldOriented(){
        swerveState = SwerveState.FIELD_ORIENTED;
    }

}