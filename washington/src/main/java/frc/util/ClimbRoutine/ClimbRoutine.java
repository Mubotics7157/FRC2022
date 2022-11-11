package frc.util.ClimbRoutine;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subsystem.Climb;

public class ClimbRoutine implements Runnable {
	boolean killFlag = false;
	boolean stopFlag = false;

	private ArrayList<ClimbCommand> routine;

	public ClimbRoutine(){
	    routine = new ArrayList<ClimbCommand>();
		routine.add(new ActuateMid());
		routine.add(new Delay(.4));
		routine.add(new ActuateHigh());
		routine.add(new Delay(.4));
		routine.add(new ClimbCommand(-642, 427685));
		SmartDashboard.putNumber("command list size", routine.size());
	}

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
		//if(!killFlag){
			for (ClimbCommand command : routine) {
				if (!Thread.interrupted()){
					command.run();
					System.out.println("*********");
				}
				else{
					//Climb.getInstance().setClimbState(ClimbState.DONE);
					break;
				}
				if (!DriverStation.isTeleop()) {
					//Climb.getInstance().setClimbState(ClimbState.DONE);
					break;
				}
			//}
			//killFlag = true;
 
		}

		//else
			//Climb.getInstance().setClimbState(ClimbState.DONE);
	}
}