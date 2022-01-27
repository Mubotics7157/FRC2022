package frc.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.Subystem.SwerveDrive.SwerveDrive;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.robot.Constants;

public class DriveToPoints extends AutoCommand {

	private ArrayList<Translation2d> points;
	private double speed;
	private boolean isReversed;
	private Rotation2d endAngle;

	public DriveToPoints(double speed, boolean isReversed, Rotation2d endingAngle, Translation2d... points) {
		this.points = new ArrayList<Translation2d>();
		this.speed = speed;
		this.isReversed = isReversed;
		this.endAngle = endingAngle;
		setBlocking(true);
		for (Translation2d point : points) {
			this.points.add(point);
		}
	}

	@Override
	public void start() {
		System.out.println("Drive To Points");
		TrajectoryConfig config = new TrajectoryConfig(speed, Constants.DriveConstants.MAX_ACCEL_AUTO);
        // add voltage constraint
		config.setReversed(isReversed);
		Translation2d end = points.get(points.size()-1);
		points.remove(end);
		Trajectory drivePath = TrajectoryGenerator.generateTrajectory(
			SwerveTracker.getInstance().getOdometry(),
			points,
			new Pose2d(end, endAngle),
			config
			);
		SwerveDrive.getInstance().setAutoPath(drivePath);
	}
	

	@Override
	public boolean isFinished() {
		return SwerveDrive.getInstance().isFinished();
	}

}