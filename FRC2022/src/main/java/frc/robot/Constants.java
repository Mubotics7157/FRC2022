package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class DiffDriveConstants{

        public static final double STICK_DEADBAND = 0.1;
        public static final double MAX_SPEED_TELE = 3;
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        // voltage needed to overcome the motor's static friction
        public static final double kS = .755;
        // voltage needed to induce a acceleration at the motor shaft
        public static final double kA = .183;
        //voltage needed to cruise at a given velocity.
        public static final double kV = 1.58;
        public static final double kP = 3.32E-2;
        // voltage needed to induce a given acceleration at the motor shaft
        public static final SimpleMotorFeedforward VELOCITY_FEED_FORWARD = new SimpleMotorFeedforward(kS, kV,kA);

        public static final double kSAngular =  1.21;
        public static final double kVAngular =  1.72;
        public static final double kAAngular =  .056;
        

        public static final int DEVICE_ID_LEFT_MASTER = 8;
        public static final int DEVICE_ID_LEFT_SLAVE = 7;
        public static final int DEVICE_ID_RIGHT_MASTER = 10;
        public static final int DEVICE_ID_RIGHT_SLAVE = 9;

        // seconds taken for motor to ramp up from 0 to setpoint
        public static final double CLOSED_LOOP_RAMP = .2;
        public static final double OPEN_LOOP_RAMP = .25;

        public static final double MAX_ANGULAR_VELOCITY = 3.25;  //MUST BE IN RAD/S

        public static final double TURN_MULTIPLIER = .05;
        public static final double WHEEL_NONLINEARITY = .05;

        public static final double TURN_P = -.008;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;

        public static final double GEAR_RATIO = 6.8888889;
    }

    public static class DriveConstants{
        
        public static final double MAX_SPEED_TELE = 3;

        public static final double GEAR_RATIO = 6.75;
        
        public static final double WHEEL_DIAMETER_INCHES = 6d;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
        public static final double WHEEL_CIRCUMFERENCE_INCHES = 2* WHEEL_DIAMETER_INCHES*Math.PI;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES)*Math.PI;

        public static final double TRACK_WIDTH_METERS = .752032621;
        public static final double TRACK_WIDTH_FEET = Units.metersToFeet(TRACK_WIDTH_METERS);
        public static final double WHEEL_BASE_METERS = .5;

        //in inches
        public static final double WHEELBASE_WIDTH = .762;
        public static final double WHEELBASE_LENGTH = .762;
        public static final double WHEELBASE_DIAMETER = Math.sqrt(Math.pow(WHEELBASE_WIDTH, 2)+Math.pow(WHEELBASE_LENGTH, 2));

        public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEEL_BASE_METERS/2,TRACK_WIDTH_METERS/2);
        public static final Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-WHEEL_BASE_METERS/2,TRACK_WIDTH_METERS/2);
        public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEEL_BASE_METERS/2,-TRACK_WIDTH_METERS/2);
        public static final Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-WHEEL_BASE_METERS/2,-TRACK_WIDTH_METERS/2);

        public static final Translation2d[] MODULE_POSITIONS = {
            FRONT_LEFT_MODULE_POSITION,
            FRONT_RIGHT_MODULE_POSITION,
            BACK_LEFT_MODULE_POSITION,
            BACK_RIGHT_MODULE_POSITION
        };
    }

    public static final class ShooterConstants{
        public static final int DEVICE_ID_SHOOTER = 3;

        public static final double GEARING = 22/12;

        public static final double TOLERANCE_RPM = 100;
        public static final double TARMAC_CLOSE_RPM = 1000;
        public static final double TARMAC_FAR_RPM = 1250;

    }

    public static final class IntakeConstants{
        public static final int DEVICE_ID_INTAKE = 5;
        public static final int DEVICE_ID_INDEXER = 10;
        public static final double OPEN_LOOP_RAMP = .2;
    }
}