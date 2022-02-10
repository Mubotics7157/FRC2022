package frc.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetShooting extends AutoCommand {
    
    boolean shoot;
    double top;
    double bot;
    public SetShooting(boolean shoot, double top, double bot){
        this.shoot = shoot;
        this.top = top;
        this.bot = bot;
    }

    @Override
    public void start() {
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
