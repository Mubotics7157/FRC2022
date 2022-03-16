package frc.auto;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.Subystem.SwerveDrive.SwerveDrive;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class AutoRoutineGenerator {
    
	private static AutoRoutine initialDrive;
	//TODO: add auto chooser options during season to quickly switch between auto routines

	public static AutoRoutine simpleLineTest(){
		TrajectoryConfig config = createConfig(.5, .5, false);
		config.setEndVelocity(0);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		//Trajectory move = PathPlanner.loadPath("Simpletest", .5, .5,true);
		Trajectory move = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0,0, Rotation2d.fromDegrees(0)),
		new Pose2d(1,0,Rotation2d.fromDegrees(0)),
		new Pose2d(1,-1,Rotation2d.fromDegrees(0)),
		new Pose2d(0,0, Rotation2d.fromDegrees(0))
		), config);
		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,false));
		return initialDrive;
	}

	public static AutoRoutine TestPath(){
		TrajectoryConfig config = createConfig(3, 3, false);
		config.setEndVelocity(0);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		initialDrive = new AutoRoutine();
		Trajectory move = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0,0, Rotation2d.fromDegrees(0)),
		new Pose2d(1.5,0, Rotation2d.fromDegrees(0))
		),config);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
		SwerveDrive.getInstance().zeroYaw();
		initialDrive.addCommands(
		new SetIntaking(false, true,false),
		new SetShooting(true,false),
		new Delay(5),
		new SetShooting(false, true),
		new SetDrivePath(move,true)
		);
		return initialDrive;
	}

	
	public static AutoRoutine TwoBallAuto(){
		TrajectoryConfig config = createConfig(3, 3, false);
		config.setEndVelocity(0);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory test = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0,0, Rotation2d.fromDegrees(0)),
		new Pose2d(2.429839726498976,-0.771075852970157, Rotation2d.fromDegrees(-3.98)),
		new Pose2d(1.392571977368473,-0.14818546675394,Rotation2d.fromDegrees(.55))
		), config);
		initialDrive = new AutoRoutine();
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
		SwerveDrive.getInstance().zeroYaw();
		initialDrive.addCommands(new SetDrivePath(test,false,PathTrigger.create(new SetIntaking(true, true, false), .05), PathTrigger.create(new SetIntaking(false, true, false), .8),PathTrigger.create(new SetIndexing(true,false),.82), PathTrigger.create(new SetIndexing(false,false), .9),PathTrigger.create(new SetShooting(true, false),.99)));
		return initialDrive;
	}

	public static AutoRoutine FiveBallAuto(){
		TrajectoryConfig config = createConfig(1, 1, false);
		config.setEndVelocity(0);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory moveToFirstBall = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0,0,Rotation2d.fromDegrees(0)),
		new Pose2d(1.266242805024287,0.137543294391451, Rotation2d.fromDegrees(-5))
		), config);
		Trajectory moveToSecondBall = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(1.266242805024287,0.137543294391451, Rotation2d.fromDegrees(-5)),
		new Pose2d(0.438413213006608,-0.032929195576166, Rotation2d.fromDegrees(14.6))		
		//new Pose2d(0.33461483505079,-0.457096641674199, Rotation2d.fromDegrees(26.470008850097656))
		), config);
		Trajectory moveBackToTarmac = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0.438413213006608,-0.032929195576166, Rotation2d.fromDegrees(14.6)),		
		new Pose2d(-1.884996237188733,-0.330717257093491, Rotation2d.fromDegrees(42.70000076293945))
		), config);
		initialDrive = new AutoRoutine();
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
		// initialDrive.addCommands(new SetDrivePath(moveToFirstBall, Rotation2d.fromDegrees(8),true),
		//  new SetDrivePath(moveToSecondBall,Rotation2d.fromDegrees(14), true),
		// new SetDrivePath(moveBackToTarmac,Rotation2d.fromDegrees(42.7) ,true));
		//new SetDrivePath(moveToSecondBall, Rotation2d.fromDegrees(26.47),true));
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