package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.util.CommonConversions;

public class Constants {
    public static class DriveConstants{


        public static final double WHEEL_DIAMETER_INCHES = 4d;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
        public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES*Math.PI;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES)*Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RAD_TELEOP = .85*Math.PI;
        public static final double MAX_TANGENTIAL_VELOCITY_TELEOP=3.5;

        public static final double MAX_TANGENTIAL_VELOCITY= 4;
        public static final double MAX_ANGULAR_VELOCITY_RAD = 2*Math.PI;

        public static final double CLOSED_LOOP_RAMP = .2;
        public static final double OPEN_LOOP_RAMP = .25;

        public static final double WHEELBASE_WIDTH = .762;
        public static final double WHEELBASE_LENGTH = .762;
        public static final double WHEELBASE_DIAMETER = Math.sqrt(Math.pow(WHEELBASE_WIDTH, 2)+Math.pow(WHEELBASE_LENGTH, 2));
        public static final double WHEELBASE_RADIUS = WHEELBASE_DIAMETER/2;

        public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
        public static final Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
        public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
        public static final Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);

        public static final Translation2d[] MODULE_POSITIONS = {
            FRONT_LEFT_MODULE_POSITION,
            FRONT_RIGHT_MODULE_POSITION,
            BACK_LEFT_MODULE_POSITION,
            BACK_RIGHT_MODULE_POSITION
        };

        public static final double MAX_SPEED_AUTO = 2;
        public static final double MAX_VOLTAGE_AUTO = 9;

        public static final double MAX_ACCEL_AUTO = 2;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_MODULE_POSITION,FRONT_RIGHT_MODULE_POSITION,BACK_LEFT_MODULE_POSITION,BACK_RIGHT_MODULE_POSITION);

        public static final double FWD_kP = 2;
        public static final double STR_kP = 2;
        public static final double THETA_kP = 3;
        public static final double THETA_kD = .015;

        public static final double FL_OFFSET = 2.724609375;
        public static final double FR_OFFSET = -111.263671875;
        public static final double BL_OFFSET = -6-2.263671875;
        public static final double BR_OFFSET = -80.439453125;
    }

    public static class ModuleConstants{
        public static final double driveKS = 0.71003;
        public static final double driveKV = 2.2783;
        public static final double driveKA = 0.25953;

        public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(driveKS,driveKV,driveKA);
        public static final double STICK_DEADBAND = 0.1;
        public static final double MAX_SPEED_TELE = 3;
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double CLOSED_LOOP_RAMP_RATE  = .2;
        public static final double OPEN_LOOP_RAMP_RATE  = .25;

        public static final double TURN_GEAR_RATIO  = 12.8;
        public static final double DRIVE_GEAR_RATIO  = 6.75;

        public static final double MOTION_PROFILE_MAX_SPEED =  CommonConversions.radPerSecToStepsPerDecisec(DriveConstants.MAX_ANGULAR_VELOCITY_RAD);
        public static final double MOTION_PROFILE_MAX_ACCEL =  CommonConversions.radPerSecSquaredToStepsPerDecisecSquared(Math.pow(DriveConstants.MAX_ANGULAR_VELOCITY_RAD, 2));


        public static final double TRACK_WIDTH_METERS = .752032621;
        public static final double TRACK_WIDTH_FEET = Units.metersToFeet(TRACK_WIDTH_METERS);

    }


    public static final class ShooterConstants{
        public static final int DEVICE_ID_SHOOTER = 3;

        public static final double TOLERANCE_RPM = 35;
        public static final double TARMAC_CLOSE_RPM = 1000;
        public static final double TARMAC_FAR_RPM = 1250;

        public static final double NORMAL_RATIO = 2;

    }

    public static final class IntakeConstants{
        public static final int DEVICE_ID_INTAKE = 32;
        public static final int DEVICE_ID_INDEXER = 21;
        public static final double OPEN_LOOP_RAMP = .2;
        public static final double INTAKE_SPEED = 1;
        public static final double INDEX_SPEED = 1;
        public static final boolean STOWED = false;
        public static final Value INTAKE_DOWN = Value.kReverse;
        public static final Value INTAKE_UP = Value.kForward;
    }

    public static final class VisionConstants{
        public static final double TARGET_HEIGHT_METERS = 0;
        public static final double CAMERA_HEIGHT_METERS = 0;
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(2);
    }
}