package frc.util.ClimbRoutine;

import frc.Subsystem.Climb;

public class ActuateHigh extends ClimbCommand {


    public ActuateHigh(){
    }

    @Override
    public void start() {
     //Climb.getInstance().toggleHighQuickRelease(true);   
    }

    @Override
    public boolean isFinished() {
       // return Climb.getInstance().isHighForward();
       return true;
    }
    
}