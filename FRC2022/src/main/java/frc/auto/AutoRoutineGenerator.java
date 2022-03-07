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
		//Trajectory move = PathPlanner.loadPath("Simpletest", .5, .5,true);
		Trajectory move = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0,0, Rotation2d.fromDegrees(0)),
		new Pose2d(2.672533772197728,-0.797712611888854,Rotation2d.fromDegrees(-26)),
		new Pose2d(2.281388799052695,1.118939840210506,Rotation2d.fromDegrees(-13.7)),
		new Pose2d(5.076671302164089,2.090779528301448, Rotation2d.fromDegrees(-31.5))
		), config);
		Trajectory goToTarmac = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0,0, Rotation2d.fromDegrees(0)),
				new Pose2d(new Translation2d(1.382069246281326, -0.116213882364911), Rotation2d.fromDegrees(0))
			), config);
		initialDrive = new AutoRoutine();
		Pose2d startPos = move.getInitialPose();
		SwerveTracker.getInstance().setOdometry(startPos);
		//initialDrive.addCommands(new Delay(2),new SetDrivePath(move,false,PathTrigger.create(new SetShooting(true,false),.1),PathTrigger.create(new SetIntaking(true), .3)));//,new SetShooting(false),new SetDrivePath(move,false,PathTrigger.create(new SetIntaking(true), .3)));
		initialDrive.addCommands(new SetIntaking(false, true,false),new SetShooting(true,false),new Delay(5),new SetShooting(false, false));
		return initialDrive;
	}

	
	public static AutoRoutine ThreeBallAuto(){
		TrajectoryConfig config = createConfig(3, 3, false);
		config.setEndVelocity(0);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		//Trajectory move = PathPlanner.loadPath("Simpletest", .5, .5,true);
		Trajectory move = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0,0, Rotation2d.fromDegrees(0)),
		new Pose2d(1,0,Rotation2d.fromDegrees(0))
		), config);
		Trajectory test = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0,0, Rotation2d.fromDegrees(0)),
		new Pose2d(2.429839726498976,-0.771075852970157, Rotation2d.fromDegrees(-3.98)),
		new Pose2d(1.392571977368473,-0.14818546675394,Rotation2d.fromDegrees(.55))
		), config);
		initialDrive = new AutoRoutine();
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
		initialDrive.addCommands(new SetDrivePath(test,true,PathTrigger.create(new SetIntaking(true, true, false), .2),
		PathTrigger.create(new SetIntaking(false, true, false), .4)), 
		new SetShooting(true, false));
		return initialDrive;
	}

	public static AutoRoutine FiveBallAuto(){
		TrajectoryConfig config = createConfig(1, 1, false);
		config.setEndVelocity(0);
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory moveToTarmac = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(1.266242805024287,0.137543294391451, Rotation2d.fromDegrees(8)),
		new Pose2d(0.227843112114361,-0.70960313240213, Rotation2d.fromDegrees(29.9)),
		new Pose2d(-0.173005021206731,0.728870181145022,Rotation2d.fromDegrees(84.0500030517)),
		new Pose2d(-0.230127509007038,-0.38584257167596, Rotation2d.fromDegrees(45.62))
		), config);
		Trajectory thirdCycle = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(3.837217778494206,2.087210653409971, Rotation2d.fromDegrees(52.4)),
		new Pose2d(-1.049,.888, Rotation2d.fromDegrees(45.86))
		), config);
		initialDrive = new AutoRoutine();
		SwerveTracker.getInstance().setOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
		initialDrive.addCommands(new SetDrivePath(moveToTarmac));
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