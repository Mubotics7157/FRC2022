package frc.util.ClimbRoutine;

import frc.Subsystem.Climb;

public class ActuateMid extends ClimbCommand {


    public ActuateMid(){
    }

    @Override
    public void start() {
     Climb.getInstance().toggleMidQuickRelease(true);   
    }

    @Override
    public boolean isFinished() {
        return Climb.getInstance().isMidForward();
    }

    
}
