package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.AbstractSubsystem;

public class Climb extends AbstractSubsystem {
    
    DoubleSolenoid midQuickRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

    WPI_TalonFX midClimb = new WPI_TalonFX(29);
    private static Climb instance = new Climb();

    private double midSetpoint;

    DigitalInput limitSwitch = new DigitalInput(1);


    private Climb(){
        super(20);
        midClimb.setSelectedSensorPosition(0);
        midClimb.configFactoryDefault();

        midClimb.setInverted(true);


        midClimb.config_kP(0, .5);
        midClimb.config_kD(0, .3);


        midClimb.configNeutralDeadband(.2);
    }

    public static Climb getInstance(){
        return instance;
    }

    public enum ClimbState{
        OFF,
        ON,
        HOMING,
        DONE
    }

    ClimbState climbState = ClimbState.OFF;

    @Override
    public void update() {
        ClimbState snapClimbState;
        synchronized(this){
            snapClimbState = climbState;
        }

        switch(snapClimbState){
            case OFF:
                holdQuickReleases();
                setMotors(0);
                break;
            case DONE:
                holdQuickReleases();
                setMotors(0);
                break;
            case HOMING:
                homeMid();
            case ON:
                updateRoutine();
                break;


        }

    }

    public synchronized void toggleMidQuickRelease(boolean on){
        midQuickRelease.set(on?Value.kForward:Value.kReverse);
    }


    public synchronized void setMotors(double mid){
        midClimb.set(mid);
    }


    private boolean withinTolerance(){
        double midTolerance =  Math.abs(midClimb.getSelectedSensorPosition()-midSetpoint);
        return (midTolerance<100);
    }


    private void holdQuickReleases(){
        midQuickRelease.set(Value.kOff);
    }

    public synchronized boolean isFinished(){
        return climbState==ClimbState.DONE;
    }

    private void updateRoutine(){
        if(!isMidForward())
            toggleMidQuickRelease(true);
        else{
            if(!limitSwitch.get())
                setMotors(-.5);
            else
                setClimbState(ClimbState.DONE);
        }

    }

    private void homeMid(){
        if(!limitSwitch.get())
            setMotors(-.5);
        else
            setClimbState(ClimbState.DONE);
    }

    public synchronized void setClimbState(ClimbState state){
        climbState = state;
    }

    public synchronized void setJog(){
        midClimb.configFactoryDefault();
        midClimb.setSelectedSensorPosition(0);
    }
    public ClimbState getClimbState(){
        return climbState;
    }

    public synchronized boolean isMidForward(){
        return midQuickRelease.get()==Value.kForward;
    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("mid height", midClimb.getSelectedSensorPosition());

        SmartDashboard.putString("Climb State", getClimbState().toString());

        SmartDashboard.putNumber("mid setpoint", midSetpoint);

        SmartDashboard.putBoolean("is finished?", isFinished());

        SmartDashboard.putBoolean("limit switch", limitSwitch.get());

        SmartDashboard.putBoolean("within tolerance?", withinTolerance());
        
    }

    @Override
    public void selfTest() {
        midClimb.configFactoryDefault();

        
    }

    
}