package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Robot;
import frc.util.AbstractSubsystem;

public class Climb extends AbstractSubsystem {
    
    WPI_TalonFX midClimb = new WPI_TalonFX(29);
    WPI_TalonFX highClimb = new WPI_TalonFX(40);
    private static Climb instance = new Climb();

    private Climb(){
        super(20);
        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();
    }

    public static Climb getInstance(){
        return instance;
    }

    @Override
    public void update() {
        setMotors(Robot.operator.getRawAxis(0), Robot.operator.getRawAxis(1));
    }


    private void setMotors(double mid, double high){
        midClimb.set(mid);
        highClimb.set(high);
    }
    @Override
    public void logData() {
        
    }

    @Override
    public void selfTest() {
        
    }

    
}
