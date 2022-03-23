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


        midClimb.configForwardSoftLimitEnable(true);
        midClimb.configReverseSoftLimitEnable(true);
        midClimb.configReverseSoftLimitThreshold(-865466);
        midClimb.configForwardSoftLimitThreshold(5000);

        midClimb.config_kP(0, .5);
        midClimb.config_kD(0, .3);

        highClimb.configForwardSoftLimitEnable(true);
        highClimb.configReverseSoftLimitEnable(true);
        highClimb.configReverseSoftLimitThreshold(-1468417);//-1302192
        highClimb.configForwardSoftLimitThreshold(5000);
        highClimb.config_kP(0, .5);
        highClimb.config_kD(0, .05);
    }
    public static Climb getInstance(){
        return instance;
    }

    public enum ClimbState{
        MANUAL,
        OFF,
        JOG,
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
            case OFF:
                setMotors(0, 0);
                break;
            case MANUAL:
                setMotors(Robot.operator.getRawAxis(1), Robot.operator.getRawAxis(5));
            break;
            case JOG:
                setMotors(Robot.operator.getRawAxis(1), Robot.operator.getRawAxis(5));
            break;
            case ROUTINE:
                updateSubRoutine();
                break;
            case DONE:
                setMotors(Robot.operator.getRawAxis(1), Robot.operator.getRawAxis(5));
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
        return (midTolerance<100&&highTolerance<100);
    }

    public synchronized boolean isFinished(){
        return climbState==ClimbState.DONE;
    }


    public synchronized void setRoutineStep(double mid, double high){
        midSetpoint = mid;
        highSetpoint = high;
    }

    public synchronized void setClimbState(ClimbState state){
        climbState = state;
    }

    public synchronized void setManual(){
       // midClimb.configFactoryDefault();
        //highClimb.configFactoryDefault();
        climbState = ClimbState.MANUAL;

    }

    public synchronized void setJog(){
        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();
        climbState = ClimbState.JOG;
    }
    public ClimbState getClimbState(){
        return climbState;
    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("mid height", midClimb.getSelectedSensorPosition());
        SmartDashboard.putNumber("high height", highClimb.getSelectedSensorPosition());

        SmartDashboard.putString("Climb State", getClimbState().toString());

        SmartDashboard.putNumber("mid setpoint", midSetpoint);
        SmartDashboard.putNumber("high setpoint", highSetpoint);

        SmartDashboard.putBoolean("is finished?", isFinished());
        
    }

    @Override
    public void selfTest() {
        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();

        
    }

    
}