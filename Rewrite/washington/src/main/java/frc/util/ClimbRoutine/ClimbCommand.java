package frc.util.ClimbRoutine;

import frc.Subsystem.Climb;

public class ClimbCommand {
    
    double mid;
    double high;
    public ClimbCommand(double mid, double high){
        this.mid = mid;
        this.high = high;
    }
    public void start(){
        Climb.getInstance().setRoutineStep(mid, high);

    }
}
