package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.AbstractSubsystem;

public class Climb extends AbstractSubsystem {
    
    WPI_TalonFX midClimb = new WPI_TalonFX(29);
    WPI_TalonFX highClimb = new WPI_TalonFX(40);
    private static Climb instance = new Climb();

    private double midSetpoint;
    private double highSetpoint;

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

    public enum ClimbState{
        MANUAL,
        ROUTINE,
        DONE
    }

    ClimbState climbState = ClimbState.MANUAL;

    @Override
    public void update() {
        ClimbState snapClimbState;
        synchronized(this){
            snapClimbState = climbState;
        }

        switch(snapClimbState){
            case MANUAL:
                setMotors(Robot.operator.getRawAxis(1), Robot.operator.getRawAxis(5));
            break;
            case ROUTINE:
                updateSubRoutine();
                break;
            case DONE:
                break;
        }
    }


    private void setMotors(double mid, double high){
        midClimb.set(mid);
        highClimb.set(high);
    }

    private void updateSubRoutine(){
        if(withinTolerance()){
            synchronized(this){
                climbState = ClimbState.DONE;
            }
        }
        midClimb.set(ControlMode.Position,midSetpoint);
        highClimb.set(ControlMode.Position,highSetpoint);
    }

    private boolean withinTolerance(){
        double midTolerance =  Math.abs(midClimb.getSelectedSensorPosition()-midSetpoint);
        double highTolerance = Math.abs( highClimb.getSelectedSensorPosition()-highSetpoint);
        return (midTolerance<4000&&highTolerance<4000);
    }

    public synchronized boolean isFinished(){
        return climbState==ClimbState.DONE;
    }


    public synchronized void setRoutineStep(double mid, double high){
        midSetpoint = mid;
        highSetpoint = high;
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
