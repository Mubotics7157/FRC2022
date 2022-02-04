package frc.auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

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

	public static AutoRoutine niceSideBlue5BallRoutine(){
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = PathPlanner.loadPath("BegBlueNiceStart", 3, 3,true);
		Trajectory move2 = PathPlanner.loadPath("BlueNiceStartSecCycle", 3, 3,true);

		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true),new SetDrivePath(move2,true));
		return initialDrive;
	}

	public static AutoRoutine niceSideBlue3BallRoutine(){
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = PathPlanner.loadPath("BegBlueNiceStart", 3, 3,true);

		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true));
		return initialDrive;
	}

	public static AutoRoutine niceSideRed5BallRoutine(){
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = PathPlanner.loadPath("AltRedStart", 3, 3,true);
		Trajectory move2 = PathPlanner.loadPath("BegRedNiceStart", 3, 3,true);

		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true),new SetDrivePath(move2,true));
		return initialDrive;

	}

	public static AutoRoutine niceSideRed3BallRoutine(){
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = PathPlanner.loadPath("AltRedStart", 3, 3,false);
		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,false));
		return initialDrive;

	}

	public static AutoRoutine BadSideBlue4BallRoutine(){
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = PathPlanner.loadPath("AltBlueBadStart4Leg1", 3, 3,false);
		Trajectory move2 = PathPlanner.loadPath("AltBlueBadStart4Leg2", 3, 3,false);
		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true),new SetDrivePath(move2,false));
		return initialDrive;
	}

	public static AutoRoutine BadSideRed4BallRoutine(){
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = PathPlanner.loadPath("RedBadStartLeg1", 3, 3,true);
		Trajectory move2 = PathPlanner.loadPath("RedBadStartLeg2", 3, 3,true);
		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,false),new SetDrivePath(move2,false));
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