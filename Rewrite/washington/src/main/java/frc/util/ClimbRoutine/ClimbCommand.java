package frc.util.ClimbRoutine;

import frc.Subsystem.Climb;
import frc.Subsystem.Climb.ClimbState;

public class ClimbCommand {
    
    double mid;
    double high;
    public ClimbCommand(double mid, double high){
        this.mid = mid;
        this.high = high;
    }
    public void start(){
        Climb.getInstance().setRoutineStep(mid, high);
        Climb.getInstance().setClimbState(ClimbState.ROUTINE);

    }

    public void run(){
        start();

        while(!Climb.getInstance().isFinished()&&!Thread.interrupted()){
        }
    }
}
