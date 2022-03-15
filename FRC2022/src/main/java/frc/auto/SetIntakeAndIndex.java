package frc.auto;

import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetIntakeAndIndex extends AutoCommand{
    boolean run;
    
    public SetIntakeAndIndex(boolean run, boolean block){
        this.run = run;
        this.setBlocking(block);
    }

    @Override
    public void start() {
        if(run)
            Serializer.getInstance().setAll();
        else    
            Serializer.getInstance().setOff();
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
