package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Threading.Threaded;

public class Climb extends Threaded {

    TalonFX midClimb;
    TalonFX highClimb;

    ClimbState climbState;
    DoubleSolenoid highClimbSolenoid; 

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

    }
    private enum ClimbState{
        OFF,
        MID_EXTEND,
        MID_RETRACT,
        HIGH_EXTEND,
        HIGH_RETRACT, 
        MANUAL_JOG
    }

    private enum ClimbHeight{
        LOW,
        MID,
        HIGH
    }

    @Override
    public void update() {
        ClimbState snapClimbState;
        synchronized(this){
            snapClimbState = climbState;
        }

        switch(snapClimbState){
            case OFF:
                SmartDashboard.putString("Climb State", "Off");
                goToZero();
            case MID_EXTEND:
                SmartDashboard.putString("Climb State", "Mid Extend");
                setMidHeight(ClimbHeight.HIGH);
            case MID_RETRACT:
                SmartDashboard.putString("Climb State", "Mid Retract");
                setMidHeight(ClimbHeight.LOW);
            case HIGH_EXTEND:
                SmartDashboard.putString("Climb State", "High Extend");
                setMidHeight(ClimbHeight.MID);
                setHighHeight(ClimbHeight.HIGH);
            case HIGH_RETRACT:
                SmartDashboard.putString("Climb State", "High Retract");
                setHighHeight(ClimbHeight.LOW);
            case MANUAL_JOG:
                SmartDashboard.putString("Climb State", "Manual Jog");
                setMotors(Robot.operator.getLeftY(), Robot.operator.getRightY());
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

}