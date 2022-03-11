package frc.auto;

import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class IndexForwards extends AutoCommand {
    boolean run;
    public IndexForwards(boolean run, boolean blocking){
        this.setBlocking(blocking);
        this.run = run;
    }

    @Override
    public void start() {
        if(run)
            Serializer.getInstance().setIntakeAndIndexBackwards();
        else
        Serializer.getInstance().setOff();
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
    
}
