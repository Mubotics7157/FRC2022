package frc.auto;
import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class RunIntake extends AutoCommand{
    
    boolean run;

    public RunIntake(boolean run){
        this.run = run;
        setBlocking(false);
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
