package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Threading.Threaded;

public class Climb extends Threaded {

    TalonFX midClimb;
    TalonFX highClimb;

    DoubleSolenoid highClimbSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5); 

    public Climb(){
        midClimb = new TalonFX(29);
        highClimb = new TalonFX(40);

        midClimb.configFactoryDefault();
        highClimb.configFactoryDefault();


        midClimb.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        midClimb.selectProfileSlot(0, 0);
        midClimb.config_kP(0, 0);
        midClimb.config_kD(0, 0);

        highClimb.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        highClimb.selectProfileSlot(0, 0);
        highClimb.config_kP(0, 0);
        highClimb.config_kD(0, 0);
        
        midClimb.configNeutralDeadband(.2);
        highClimb.configNeutralDeadband(.2);

        midClimb.configReverseSoftLimitEnable(true);
        midClimb.configReverseSoftLimitThreshold(-1, 20);
        midClimb.configPeakOutputForward(1);
        midClimb.configPeakOutputReverse(-1);
        midClimb.setSelectedSensorPosition(0);

        midClimb.setNeutralMode(NeutralMode.Brake);

    }
    private enum ClimbState{
        OFF,
        MID_EXTEND,
        MID_RETRACT,
        HIGH_EXTEND,
        HIGH_RETRACT, 
        MANUAL_JOG,
        HOMING
    }

    private enum ClimbHeight{
        LOW,
        MID,
        HIGH
    }

    ClimbState climbState = ClimbState.OFF;
    @Override
    public void update(){
        ClimbState snapClimbState;
        synchronized(this){
            snapClimbState = climbState;
        }

        switch(snapClimbState){
            case OFF:
                SmartDashboard.putString("Climb State", "Off");
                goToZero();
                break;
            case MID_EXTEND:
                SmartDashboard.putString("Climb State", "Mid Extend");
                setMidHeight(ClimbHeight.HIGH);
                break;
            case MID_RETRACT:
                SmartDashboard.putString("Climb State", "Mid Retract");
                setMidHeight(ClimbHeight.LOW);
                break;
            case HIGH_EXTEND:
                SmartDashboard.putString("Climb State", "High Extend");
                setMidHeight(ClimbHeight.MID);
                setHighHeight(ClimbHeight.HIGH);
                break;
            case HIGH_RETRACT:
                SmartDashboard.putString("Climb State", "High Retract");
                setHighHeight(ClimbHeight.LOW);
                break;
            case MANUAL_JOG:
                SmartDashboard.putString("Climb State", "Manual Jog");
                setMotors(Robot.operator.getRawAxis(2), Robot.operator.getRawAxis(0));
                break;
            case HOMING:
                SmartDashboard.putString("Climb State","Homing");
                setMotors(Robot.operator.getRawAxis(2), Robot.operator.getRawAxis(0));
                break;
        }
        SmartDashboard.putNumber("mid height", midClimb.getSelectedSensorPosition());
        SmartDashboard.putNumber("high height", highClimb.getSelectedSensorPosition());
    }

    private void goToZero(){
        midClimb.set(ControlMode.Position, 0);
        highClimb.set(ControlMode.Position, 0);
    }

    private void setMidHeight(ClimbHeight height){
        if(height==ClimbHeight.LOW)
            midClimb.set(ControlMode.Position, 500);
        else if(height==ClimbHeight.MID)
            midClimb.set(ControlMode.Position, 1000);
        else if(height==ClimbHeight.HIGH) 
            midClimb.set(ControlMode.Position, 1500);
    }

    private void setHighHeight(ClimbHeight height){
        if(height==ClimbHeight.LOW)
            highClimb.set(ControlMode.Position, 500);
        else if(height==ClimbHeight.MID)
            highClimb.set(ControlMode.Position, 1000);
        else if(height==ClimbHeight.HIGH) 
            highClimb.set(ControlMode.Position, 1500);
    }

    private void setMotors(double midSpeed, double highSpeed){
        midClimb.set(ControlMode.PercentOutput,midSpeed);
        highClimb.set(ControlMode.PercentOutput,highSpeed);
    }


    public synchronized void toggleClimbSolenoid(){
        highClimbSolenoid.set(Value.kForward);
    }

    public synchronized void setJogging(){
        climbState = ClimbState.MANUAL_JOG;
    }

    public synchronized void setMidExtend(){
        climbState = ClimbState.MID_EXTEND;
    }

    public synchronized void setMidRetract(){
        climbState = ClimbState.MID_RETRACT;
    }

    public synchronized void setHighExtend(){
        climbState = ClimbState.HIGH_EXTEND;
    }

    public synchronized void setHighRetract(){
        climbState = ClimbState.HIGH_RETRACT;
    }

    public synchronized void setManual(){
        climbState = ClimbState.MANUAL_JOG;
    }

     public synchronized void setOff(){
        climbState = ClimbState.OFF;
    }
    public synchronized void setHoming(){
        midClimb.configFactoryDefault();
        midClimb.setNeutralMode(NeutralMode.Brake);
        highClimb.setNeutralMode(NeutralMode.Brake);
        climbState = ClimbState.HOMING;
    }
}