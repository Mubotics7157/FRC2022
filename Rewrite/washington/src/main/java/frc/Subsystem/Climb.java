package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.AbstractSubsystem;

public class Climb extends AbstractSubsystem {
    
    WPI_TalonFX midClimb = new WPI_TalonFX(29);
    WPI_TalonFX highClimb = new WPI_TalonFX(40);
    private static Climb instance = new Climb();

    private Climb(){
        super(20);
        midClimb.setSelectedSensorPosition(0);
        highClimb.setSelectedSensorPosition(0);
        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();
    }

    public static Climb getInstance(){
        return instance;
    }

    @Override
    public void update() {
        setMotors(Robot.operator.getRawAxis(1), Robot.operator.getRawAxis(5));
    }


    private void setMotors(double mid, double high){
        midClimb.set(mid);
        highClimb.set(high);
    }
    @Override
    public void logData() {
        SmartDashboard.putNumber("mid height", midClimb.getSelectedSensorPosition());
        SmartDashboard.putNumber("high height", highClimb.getSelectedSensorPosition());
        
    }

    @Override
    public void selfTest() {
        
    }

    
}
