package frc.auto;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoRoutine implements Runnable {

	private ArrayList<AutoCommand> routine = new ArrayList<AutoCommand>();

	synchronized public void addCommands(AutoCommand... commands) {
		routine.addAll(Arrays.asList(commands));
	}

	synchronized public void addRoutines(AutoRoutine... routines) {
		for (AutoRoutine r : routines) {
			routine.addAll(r.routine);
		}
	}

	@Override
	synchronized public void run() {
		try {
			for (AutoCommand command : routine) {
				if (Thread.interrupted()) throw new InterruptedException();
				command.run();
				if (!DriverStation.isAutonomous()) {
					break;
				}
			}
		}
 
		catch (InterruptedException ex) {
			
		}
	}
}