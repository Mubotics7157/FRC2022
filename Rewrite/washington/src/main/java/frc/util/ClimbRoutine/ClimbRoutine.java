package frc.util.ClimbRoutine;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subsystem.Climb;
import frc.Subsystem.Climb.ClimbState;

public class ClimbRoutine implements Runnable {
	boolean killFlag = false;
	boolean stopFlag = false;

	private ArrayList<ClimbCommand> routine = new ArrayList<ClimbCommand>();

	synchronized public void addCommands(ClimbCommand... commands) {
		routine.addAll(Arrays.asList(commands));
		System.out.println(routine.size());
	}

	synchronized public void addRoutines(ClimbRoutine... routines) {
		for (ClimbRoutine r : routines) {
			routine.addAll(r.routine);
		}
	}

	@Override
	synchronized public void run() {
		SmartDashboard.putBoolean("stop climb sequence?", killFlag);
		if(!killFlag){
			for (ClimbCommand command : routine) {
				if (!Thread.interrupted())
					command.run();
				else
					break;
				if (!DriverStation.isTeleop()) {
					break;
				}
			}
			killFlag = true;
 
		}
	}
}