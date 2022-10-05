package frc.util.ClimbRoutine;

import frc.Subsystem.Climb;
import frc.Subsystem.Climb.ClimbState;

public class ClimbCommand {
    
    double mid;
    double high;
    boolean useSwitch = false;
    public ClimbCommand(double mid, double high){
        this.mid = mid;
        this.high = high;
        this.useSwitch = false;
    }

    public ClimbCommand(double mid, double high, boolean useSwitch){
        this.mid = mid;
        this.high = high;
        this.useSwitch = useSwitch;
    }

    public ClimbCommand(){

    }
    public void start(){
        Climb.getInstance().setRoutineStep(mid, high,useSwitch);
        Climb.getInstance().setClimbState(ClimbState.ROUTINE);

    }

    public boolean isFinished(){
        return Climb.getInstance().isFinished();
    }


    public void run(){
        start();

        while(!isFinished()&&!Thread.interrupted()){
        }
    }

}
