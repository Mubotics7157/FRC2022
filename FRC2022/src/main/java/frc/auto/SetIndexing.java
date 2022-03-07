package frc.auto;

import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetIndexing extends AutoCommand{
    boolean index;

    public SetIndexing(boolean index,boolean blocking){
        this.setBlocking(blocking);
        this.index = index;
    }
    @Override
    public void start() {
        if(index)
            Serializer.getInstance().setIndexing();
        else
            Serializer.getInstance().setOff();
    }
    
    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
