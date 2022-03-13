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
        /*if(run)
        else
            Serializer.getInstance().setOff();
        if(down)
        else
            Serializer.getInstance().toggleIntake(false);
            */
            /*if(down)
            else
            //Serializer.getInstance().setIntakeAndIndexBackwards();
            */
            Serializer.getInstance().toggleIntake(true);
            Serializer.getInstance().setAll();
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
