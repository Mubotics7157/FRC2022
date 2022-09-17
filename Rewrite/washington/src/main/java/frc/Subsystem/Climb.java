package frc.Subsystem;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.AbstractSubsystem;

public class Climb extends AbstractSubsystem {
    DigitalInput magSensor = new DigitalInput(0);
    //True if not detected and false if it is
    
    DoubleSolenoid midQuickRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    DoubleSolenoid highQuickRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,6,7);

    WPI_TalonFX midClimb = new WPI_TalonFX(29);
    WPI_TalonFX highClimb = new WPI_TalonFX(40);
    private static Climb instance = new Climb();

    private double midSetpoint;

    //DigitalInput limitSwitch = new DigitalInput(1);
    boolean useLimitSwitch = false;

    private double highSetpoint;

    private Climb(){
        super(20);
        midClimb.setSelectedSensorPosition(0);
        highClimb.setSelectedSensorPosition(0);
        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();

        midClimb.setInverted(true);
        highClimb.setInverted(true);


        midClimb.config_kP(0, .5);
        midClimb.config_kD(0, .3);

        highClimb.config_kP(0, .5);
        highClimb.config_kD(0, .05);

        midClimb.configNeutralDeadband(.2);
        highClimb.configNeutralDeadband(.2);
    }
    public static Climb getInstance(){
        return instance;
    }

    public enum ClimbState{
        MANUAL,
        OFF,
        JOG,
        ROUTINE,
        RESET,
        ZERO,
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
                holdQuickReleases();
                setMotors(0);
                break;
            case MANUAL:
                setMotors(0);
            break;
            case JOG:
                setMotors(Robot.operator.getRawAxis(1));
            break;
            case ROUTINE:
                updateSubRoutine(1);
                break;
            case ZERO:
                zeroClimb();
                break;
            case RESET:
               resetClimb();
                break;
            case DONE:
                setMotors(Robot.operator.getRawAxis(1));
                break;
        }

        //if(midQuickRelease.get()==Value.kForward)

    }

    public synchronized void toggleMidQuickRelease(boolean on){
        midQuickRelease.set(on?Value.kForward:Value.kReverse);
    }

    public synchronized void toggleHighQuickRelease(boolean on){
        highQuickRelease.set(on?Value.kForward:Value.kReverse);
    }

    private void setMotors(double mid){
        midClimb.set(mid);
    }

    private void updateSubRoutine(int button){
        if(withinTolerance()){
            synchronized(this){
                climbState = ClimbState.DONE;
            }
        }

        else{
             if(Robot.operator.getRawButton(button) && highQuickRelease.get() != Value.kReverse)
        highQuickRelease.set(Value.kReverse);
            else if(Robot.operator.getRawButton(button) && highQuickRelease.get() ==Value.kReverse)
        midClimb.set(ControlMode.Position, 160000);
        }
        }

    private void resetClimb(){
        if(magSensor.get() && highQuickRelease.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 600000) > 1000)
            midClimb.set(ControlMode.Position, 600000);
   else if(!magSensor.get() && highQuickRelease.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 600000) < 1000){
     highQuickRelease.set(Value.kForward);
    }
    }
    
    private void zeroClimb(){
        //if magnet is not detected then go down at 40 percent speed until it is detected
        //If it is detected then make the encoder reading 0
        if(magSensor.get())
        midClimb.set(ControlMode.PercentOutput, -0.4);
        else if(!magSensor.get()){
        midClimb.set(ControlMode.PercentOutput, 0);
        midClimb.setSelectedSensorPosition(0);
        }
        //just ctrl c ctrl v resetClimb but got rid of the last line
        //should be called in test periodic or something to zero in a systems check
    }

    private boolean withinTolerance(){
        double midTolerance =  Math.abs(midClimb.getSelectedSensorPosition()-midSetpoint);
        double highTolerance = Math.abs( highClimb.getSelectedSensorPosition()-highSetpoint);
        return (midTolerance<100&&highTolerance<100);
    }

    private boolean withinHighTolerance(){

        double highTolerance = Math.abs( highClimb.getSelectedSensorPosition()-highSetpoint);
        return (highTolerance<100);
    }

    private void holdQuickReleases(){
        midQuickRelease.set(Value.kOff);
        highQuickRelease.set(Value.kOff);
    }

    public synchronized boolean isFinished(){
        return climbState==ClimbState.DONE;
    }


    public synchronized void setRoutineStep(double mid, double high){
        midSetpoint = mid;
        highSetpoint = high;
        this.useLimitSwitch = false;
    }

    public synchronized void setRoutineStep(double mid, double high, boolean useLimitSwitch){
        midSetpoint = mid;
        highSetpoint = high;
        this.useLimitSwitch = useLimitSwitch;
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
        //midClimb.setSelectedSensorPosition(0);
        highClimb.setSelectedSensorPosition(0);
        climbState = ClimbState.JOG;
    }
    public ClimbState getClimbState(){
        return climbState;
    }

    public synchronized void setZero(){
        climbState = ClimbState.ZERO;
    }

    public synchronized void setReset(){
        climbState = ClimbState.RESET;
    }

    public synchronized boolean isMidForward(){
        return midQuickRelease.get()==Value.kForward;
    }

    public synchronized boolean isHighForward(){
        return highQuickRelease.get()==Value.kForward;
    }
    @Override
    public void logData() {
        SmartDashboard.putNumber("mid height", midClimb.getSelectedSensorPosition());
        SmartDashboard.putNumber("high height", highClimb.getSelectedSensorPosition());

        SmartDashboard.putString("Climb State", getClimbState().toString());

        SmartDashboard.putNumber("mid setpoint", midSetpoint);
        SmartDashboard.putNumber("high setpoint", highSetpoint);

        SmartDashboard.putBoolean("is finished?", isFinished());

        //SmartDashboard.putBoolean("limit switch", limitSwitch.get());
        //SmartDashboard.putBoolean("use limit switch", useLimitSwitch);

        SmartDashboard.putBoolean("within tolerance?", withinTolerance());
        
        SmartDashboard.putBoolean("condition 1 met", magSensor.get() && highQuickRelease.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 600000) > 1000);
        SmartDashboard.putBoolean("condition 2 met", !magSensor.get() && highQuickRelease.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 600000) < 1000);
        SmartDashboard.putString("mid solenoid", midQuickRelease.get().toString());
        SmartDashboard.putString("high solenoid", highQuickRelease.get().toString());
        
    }

    @Override
    public void selfTest() {
        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();

        
    }

    
}