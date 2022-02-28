package frc.auto;

import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetIntaking extends AutoCommand {
    
    boolean run;
    public SetIntaking(boolean runIntake){
        run = runIntake;
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
