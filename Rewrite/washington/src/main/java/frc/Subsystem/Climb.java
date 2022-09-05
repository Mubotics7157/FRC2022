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
                setMotors(0, 0);
                break;
            case MANUAL:
                setMotors(0, 0);
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

        //if(midQuickRelease.get()==Value.kForward)

    }

    public synchronized void toggleMidQuickRelease(boolean on){
        midQuickRelease.set(on?Value.kForward:Value.kReverse);
    }

    public synchronized void toggleHighQuickRelease(boolean on){
        highQuickRelease.set(on?Value.kForward:Value.kReverse);
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
            else{
                midClimb.set(ControlMode.Position,midSetpoint);
                highClimb.set(ControlMode.Position,highSetpoint);

            }
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
        midClimb.setSelectedSensorPosition(0);
        highClimb.setSelectedSensorPosition(0);
        climbState = ClimbState.JOG;
    }
    public ClimbState getClimbState(){
        return climbState;
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


        SmartDashboard.putBoolean("within tolerance?", withinTolerance());
        
    }

    @Override
    public void selfTest() {
        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();

        
    }

    
}