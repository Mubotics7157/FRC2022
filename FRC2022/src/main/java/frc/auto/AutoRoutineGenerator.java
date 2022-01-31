package frc.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class AutoRoutineGenerator {
    
	private static AutoRoutine initialDrive;
	//TODO: add auto chooser options during season to quickly switch between auto routines

	public static AutoRoutine simpleLineTest(){
		TrajectoryConfig config = createConfig(.5, .5, false);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = TrajectoryGenerator.generateTrajectory(
			List.of(
			new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			new Pose2d(2,0,Rotation2d.fromDegrees(0))
		),
		config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true));
		return initialDrive;
	}

	public static AutoRoutine bouncePath(){
		TrajectoryConfig config = createConfig(1.5, 1.5, false);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory part1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90)))
		), config);

		Trajectory part2 = TrajectoryGenerator.generateTrajectory(
			List.of(
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-250))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-240))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(-180))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-90)))
		), config);

		Trajectory part3 = TrajectoryGenerator.generateTrajectory(List.of(
			    new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90)))
		), config);
		Trajectory part4 = TrajectoryGenerator.generateTrajectory(List.of(
			    new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-180)))
		), config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(part1, true),new SetDrivePath(part2,true),new SetDrivePath(part3,true), new SetDrivePath(part4,true));
		return initialDrive;
	}

	public static AutoRoutine niceSideBlue2BallRoutine(){
		TrajectoryConfig config = createConfig(.5, .5, false);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = TrajectoryGenerator.generateTrajectory(
			List.of(
			new Pose2d(7.814,2.482,new Rotation2d(0.045, 0.529)),
			new Pose2d(7.632,0.365,new Rotation2d(0.272, 0.711)),
			new Pose2d(6.377,0.607,new Rotation2d(0.955, -0.519)),
			new Pose2d(5.076,1.832,new Rotation2d(0.499, -1.165))

		),
		config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(7.723,-5.899),new Rotation2d(.121, 1.573));
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true));
		return initialDrive;
	}


    private static TrajectoryConfig createConfig(double velocity, double accel, boolean reversed) {
		TrajectoryConfig config = new TrajectoryConfig(velocity, accel);
		config.setKinematics(Constants.DriveConstants.SWERVE_KINEMATICS);
        //voltage constraint?
        SwerveDriveKinematicsConstraint driveConstraint = new SwerveDriveKinematicsConstraint(DriveConstants.SWERVE_KINEMATICS, DriveConstants.MAX_SPEED_AUTO);
		config.addConstraint(driveConstraint);
		config.setReversed(reversed);
		return config;
	}

    private static TrajectoryConfig createConfig(double velocity, double accel, double endVel, boolean reversed) {
		TrajectoryConfig config = new TrajectoryConfig(velocity, accel);
        //voltage constraint?
		config.setKinematics(Constants.DriveConstants.SWERVE_KINEMATICS);
        SwerveDriveKinematicsConstraint driveConstraint = new SwerveDriveKinematicsConstraint(DriveConstants.SWERVE_KINEMATICS, DriveConstants.MAX_SPEED_AUTO);
		config.addConstraint(driveConstraint);
		config.setReversed(reversed);
		config.setEndVelocity(endVel);
		return config;
	}
}