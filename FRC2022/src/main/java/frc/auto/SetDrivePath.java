package frc.auto;

import java.util.ArrayList;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.Subystem.SwerveDrive.SwerveDrive;


public class SetDrivePath extends AutoCommand {

	private Trajectory robotPath;
	private ArrayList<PathTrigger> triggers = new ArrayList<>();

	public SetDrivePath(Trajectory robotPath) {
		this(robotPath, true);
	}

	public SetDrivePath(Trajectory robotPath, boolean isBlocking) {
		System.out.println("Set Drive Path");
		this.robotPath = robotPath;
		this.setBlocking(isBlocking);
	}
	
	public SetDrivePath(Trajectory robotPath, boolean isBlocking, PathTrigger... triggers) {
		System.out.println("Set Drive Path");
		this.robotPath = robotPath;
		this.setBlocking(isBlocking);
		for (PathTrigger trigger : triggers) {
			this.triggers.add(trigger);
		}
	}

	@Override
	public boolean isFinished() {
		if(SwerveDrive.getInstance().isFinished()) {
			System.out.println("pathcomplete");
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		SwerveDrive.getInstance().setAutoPath(robotPath, triggers);
	}

}