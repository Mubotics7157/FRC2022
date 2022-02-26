package frc.auto;

import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetShooting extends AutoCommand {
    
    boolean shoot;
    double topSpeed;
    double botSpeed;
    
    public SetShooting(boolean shoot){
        this.shoot = shoot;
    }

    public SetShooting(boolean shoot, double top, double bot){
        this.shoot = shoot;
        this.topSpeed = top;
        this.botSpeed = bot;
    }

    @Override
    public void start() {
        if(shoot)
            Serializer.getInstance().setArbitrary(topSpeed, botSpeed);
        else
            Serializer.getInstance().setArbitrary(0, 0);
    }

    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
