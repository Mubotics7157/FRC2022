package frc.Subystem.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.util.Threading.Threaded;
public class SwerveDrive extends Threaded{
    Pose2d[] modulePoses = {
        new Pose2d(),
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };
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
    double [] angles= new double[4];
    double[]speeds = new double[4];
    ModuleHelper helper = new ModuleHelper();
    private final AnalogGyro navX = new AnalogGyro(0);
    AnalogGyroSim navXSim = new AnalogGyroSim(navX);
    Field2d field = new Field2d();
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.SWERVE_KINEMATICS, Rotation2d.fromDegrees(navX.getAngle()));
    double yawVal = 0;
    private static SwerveDrive instance;

    public static SwerveDrive getInstance(){
        if(instance==null)
            instance = new SwerveDrive();
        return instance;
    }

    @Override
    public void update() {
        updateOdometry();
        updateFieldOriented(Robot.operator.getLeftX(), Robot.operator.getLeftY(), Robot.operator.getRightX());

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
    SwerveModuleState[] moduleStates = {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };

    var chassisSpeed = Constants.DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(moduleStates);
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

    yawVal += chassisRotationSpeed * 0.02;
    navXSim.setAngle(-Units.radiansToDegrees(yawVal));
    }

    private void updateOdometry(){
        odometry.update(Rotation2d.fromDegrees(navX.getAngle()), frontLeft.getState(),frontRight.getState(),backLeft.getState(),backRight.getState());
        SmartDashboard.putData(field);
        field.setRobotPose(odometry.getPoseMeters());
        for (int i = 0; i < DriveConstants.MODULE_POSITIONS.length; i++) {
            Translation2d modulePositionFromChassis = DriveConstants.MODULE_POSITIONS[i]
            .rotateBy(Rotation2d.fromDegrees(navX.getAngle()))
            .plus(odometry.getPoseMeters().getTranslation());

            modulePoses[i] = new Pose2d(modulePositionFromChassis, modules[i].getState().angle.plus(odometry.getPoseMeters().getRotation()));
        }
       field.getObject("Modules").setPoses(modulePoses);
    }
}