package frc.auto;

import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;

public class SetShooting extends AutoCommand {
    
    boolean shoot;
    double topSpeed = 1250;
    double botSpeed = 1350;
    
    public SetShooting(boolean shoot, boolean blocking){
        this.shoot = shoot;
        this.setBlocking(blocking);
    }

    public SetShooting(boolean shoot, double top, double bot){
        this.shoot = shoot;
        this.topSpeed = top;
        this.botSpeed = bot;
        Serializer.getInstance().setShooterSpeed(top,bot);
    }

    @Override
    public void start() {
        if(shoot)
        {
            Serializer.getInstance().setShooting();
            System.out.println("shooting");
        }
        else
            Serializer.getInstance().setOff();
    }


    @Override
    public boolean isFinished() {
        return SwerveDrive.getInstance().isFinished();
    }
}
