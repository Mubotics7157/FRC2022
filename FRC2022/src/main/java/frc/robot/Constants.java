package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.util.CommonConversions;

public class Constants {
    public static class DriveConstants{


        public static final double WHEEL_DIAMETER_INCHES = 4d;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
        public static final double WHEEL_CIRCUMFERENCE_INCHES = 2* WHEEL_DIAMETER_INCHES*Math.PI;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES)*Math.PI;
        public static final double STICK_DEADBAND = 0.1;

        public static final double MAX_ANGULAR_VELOCITY_RAD = 16*Math.PI;
        public static final double MAX_TANGENTIAL_VELOCITY = 5;

        public static final double CLOSED_LOOP_RAMP = .2;
        public static final double OPEN_LOOP_RAMP = .25;
        

        public static final double TRACK_WIDTH_METERS = .752032621;
        public static final double TRACK_WIDTH_FEET = Units.metersToFeet(TRACK_WIDTH_METERS);
        public static final double WHEEL_BASE_METERS = .5;

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

        public static final double MAX_SPEED_AUTO = 2;
        public static final double MAX_VOLTAGE_AUTO = 9;

        public static final double MAX_ACCEL_AUTO = 2;


        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_MODULE_POSITION,FRONT_RIGHT_MODULE_POSITION,BACK_LEFT_MODULE_POSITION,BACK_RIGHT_MODULE_POSITION);

    }

    public static class ModuleConstants{
        public static final double driveKS = 0.6;
        public static final double driveKV = 2.9;
        public static final double driveKA = 0.4;
        public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(driveKS,driveKV,driveKA);
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
        

        public static final double steerKS = 0;
        public static final double steerKV = .15;
        public static final double steerKA = .04;
        public static final SimpleMotorFeedforward STEER_FEEDFORWARD = new SimpleMotorFeedforward(steerKS,steerKV,steerKA);

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

        public static final double TOLERANCE_RPM = 20;
        public static final double TARMAC_CLOSE_RPM = 1000;
        public static final double TARMAC_FAR_RPM = 1250;

    }

    public static final class IntakeConstants{
        public static final int DEVICE_ID_INTAKE = 18;
        public static final int DEVICE_ID_INDEXER = 7;
        public static final double OPEN_LOOP_RAMP = .2;
        public static final double INTAKE_SPEED = -1;
        public static final double INDEX_SPEED = -.3;
        public static final boolean STOWED = false;
    }


// courtesy of team 6328
public static final class FieldConstants {

  // Field dimensions
  public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
  public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
  public static final double hangarLength = Units.inchesToMeters(128.75);
  public static final double hangarWidth = Units.inchesToMeters(116.0);

  // Vision target
  public static final double visionTargetDiameter =
      Units.inchesToMeters(4.0 * 12.0 + 5.375);
  public static final double visionTargetHeightLower =
      Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
  public static final double visionTargetHeightUpper =
      visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape

  // Dimensions of hub and tarmac
  public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
  public static final Translation2d hubCenter =
      new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
  public static final double tarmacDiameter = Units.inchesToMeters(219.25); // Inner diameter
  public static final double tarmacFullSideLength =
      tarmacDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
  public static final double tarmacMarkedSideLength =
      Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
  public static final double tarmacMissingSideLength =
      tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff

  // Reference rotations (angle from hub to each reference point)
  public static final Rotation2d referenceARotation =
      Rotation2d.fromDegrees(180.0).minus(centerLineAngle)
          .plus(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d referenceBRotation =
      referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceCRotation =
      referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceDRotation =
      referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));

  // Reference points (centered of the sides of the tarmac if they formed a complete octagon)
  public static final Pose2d referenceA =
      new Pose2d(hubCenter, referenceARotation).transformBy(
          new Transform2d(new Translation2d(tarmacDiameter/2,0),new Rotation2d()));
  public static final Pose2d referenceB =
      new Pose2d(hubCenter, referenceBRotation).transformBy(
          new Transform2d(new Translation2d(tarmacDiameter/2,0),new Rotation2d()));
  public static final Pose2d referenceC =
      new Pose2d(hubCenter, referenceCRotation).transformBy(
          new Transform2d(new Translation2d(tarmacDiameter/2,0),new Rotation2d()));
  public static final Pose2d referenceD =
      new Pose2d(hubCenter, referenceDRotation).transformBy(
          new Transform2d(new Translation2d(tarmacDiameter/2, 0), new Rotation2d()));

  // Cargo points
  public static final double cornerToCargoY = Units.inchesToMeters(15.56);
  public static final double referenceToCargoY =
      (tarmacFullSideLength / 2.0) - cornerToCargoY;
  public static final double referenceToCargoX = Units.inchesToMeters(40.44);
  public static final Pose2d cargoA = referenceA.transformBy(
      new Transform2d(new Translation2d(referenceToCargoX,-referenceToCargoY),new Rotation2d()));
  public static final Pose2d cargoB = referenceA.transformBy(
      new Transform2d(new Translation2d(referenceToCargoX,referenceToCargoY),new Rotation2d()));
  public static final Pose2d cargoC = referenceB.transformBy(
      new Transform2d(new Translation2d(referenceToCargoX,referenceToCargoY),new Rotation2d()));
  public static final Pose2d cargoD = referenceC.transformBy(
      new Transform2d(new Translation2d(referenceToCargoX,-referenceToCargoY),new Rotation2d()));
  public static final Pose2d cargoE = referenceD.transformBy(
      new Transform2d(new Translation2d(referenceToCargoX,-referenceToCargoY),new Rotation2d()));
  public static final Pose2d cargoF = referenceD.transformBy(
      new Transform2d(new Translation2d(referenceToCargoX,referenceToCargoY),new Rotation2d()));

    }
}