package frc.auto;

import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetIntaking extends AutoCommand {
    
    boolean run;
    boolean down;
    public SetIntaking(boolean runIntake, boolean down, boolean blocking){
        this.setBlocking(blocking);
        run = runIntake;
        this.down = down;
    }

    @Override
    public void start() {
        if(run)
            Serializer.getInstance().setIntakeBackwards();
        else
            Serializer.getInstance().setOff();
        if(down)
            Serializer.getInstance().toggleIntake(true);
        else
            Serializer.getInstance().toggleIntake(false);
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
