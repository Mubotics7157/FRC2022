package frc.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetIntaking extends AutoCommand {
    
    boolean down;
    boolean run;
    public SetIntaking(boolean intakeDown, boolean runIntake){
        down = intakeDown;
        run = runIntake;
    }

    @Override
    public void start() {
        SmartDashboard.putBoolean("down and running?", (down&&run));
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
