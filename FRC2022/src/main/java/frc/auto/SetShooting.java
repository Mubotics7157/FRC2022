package frc.auto;

import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetShooting extends AutoCommand {
    
    boolean shoot;
    public SetShooting(boolean shoot){
        this.shoot = shoot;
    }

    @Override
    public void start() {
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
