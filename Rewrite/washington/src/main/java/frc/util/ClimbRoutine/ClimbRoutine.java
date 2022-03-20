package frc.util.ClimbRoutine;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;

public class ClimbRoutine implements Runnable {

	private ArrayList<ClimbCommand> routine = new ArrayList<ClimbCommand>();

	synchronized public void addCommands(ClimbCommand... commands) {
		routine.addAll(Arrays.asList(commands));
	}

	synchronized public void addRoutines(ClimbRoutine... routines) {
		for (ClimbRoutine r : routines) {
			routine.addAll(r.routine);
		}
	}

	@Override
	synchronized public void run() {
		try {
			for (ClimbCommand command : routine) {
				if (Thread.interrupted()) throw new InterruptedException();
				command.run();
				if (!DriverStation.isTeleop()) {
					break;
				}
			}
		}
 
		catch (InterruptedException ex) {
			
		}
	}
}