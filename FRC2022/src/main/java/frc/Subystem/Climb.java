package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Threading.Threaded;

public class Climb extends Threaded {
    private TalonFX climbMotor;
    private TalonFX highMotor;
    private ClimbState climbState = ClimbState.OFF;

    private static Climb instance;

    DoubleSolenoid climbSolenoid;

    public Climb(){
        climbMotor = new TalonFX(40);
        climbMotor.setSelectedSensorPosition(0);
        climbMotor.setInverted(InvertType.InvertMotorOutput);
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        climbMotor.configFactoryDefault();
        climbMotor.configPeakOutputForward(1);
        climbMotor.configPeakOutputReverse(-1);


        climbMotor.configForwardSoftLimitEnable(true);
        climbMotor.configReverseSoftLimitEnable(true);
        climbMotor.configForwardSoftLimitThreshold(776071);
        climbMotor.configReverseSoftLimitThreshold(-1);

        climbMotor.setNeutralMode(NeutralMode.Brake);

        climbMotor.configMotionCruiseVelocity(200,30);
        climbMotor.configMotionAcceleration(200,30);

        climbMotor.selectProfileSlot(0, 0);

        climbMotor.config_kP(0, .001);


        highMotor = new TalonFX(29);
        highMotor.setSelectedSensorPosition(0);
        highMotor.setInverted(InvertType.InvertMotorOutput);
        highMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        highMotor.configFactoryDefault();
        highMotor.configPeakOutputForward(1);
        highMotor.configPeakOutputReverse(-1);
        highMotor.setNeutralMode(NeutralMode.Brake);

        highMotor.configMotionCruiseVelocity(200,30);
        highMotor.configMotionAcceleration(200,30);
        highMotor.configForwardSoftLimitEnable(true);
        highMotor.configReverseSoftLimitEnable(true);
        highMotor.configForwardSoftLimitThreshold(11000);
        highMotor.configReverseSoftLimitThreshold(-1);

    }   

    public static Climb getInstance(){
        if(instance==null)
            instance = new Climb();
        return instance;
    }

    private enum ClimbState{
        EXTEND,
        RETRACT,
        HOMING,
        HIGH_EXTEND,
        HIGH_RETRACT,
        OFF
    }

    @Override
    public void update() {
        ClimbState snapClimbState;
        synchronized(this){
            snapClimbState = climbState ;
        }

        switch(snapClimbState){
            case EXTEND:
                SmartDashboard.putString("climb state", "Extending");
                //goUp();
            climbHigh(.3);
                break;
            case RETRACT:
                SmartDashboard.putString("climb state", "Retracting");
            climbHigh(-.3);
                //goDown();
                break;
            case HOMING:
                SmartDashboard.putString("climb state", "Homing");
                climb(Robot.driver.getRawAxis(5));
                break;
            case HIGH_EXTEND:
                climb(-.3);
                SmartDashboard.putString("climb state", "High Extend");
                break;
            case HIGH_RETRACT:
                climb(.3);
                SmartDashboard.putString("climb state", "High Retract");
                break;
            case OFF:
                SmartDashboard.putString("climb state", "Off");
                break;
        }
    }

    private void climb(double speed){
        climbMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("climb height", climbMotor.getSelectedSensorPosition());
    }

    private void climbHigh(double speed){
        highMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("high climb height", highMotor.getSelectedSensorPosition());
    }

    private void goUp(){
        climbMotor.set(ControlMode.MotionMagic  ,775000);
    }

    private void goHighUp(){
        highMotor.set(ControlMode.MotionMagic  ,775000);
    }

    private void goDown(){
        climbMotor.set(ControlMode.MotionMagic,500);
    }

    private void goHighDown(){
        highMotor.set(ControlMode.MotionMagic,500);
    }

    public synchronized void setExtending(){
        climbState = ClimbState.EXTEND;
    }

    public synchronized void setRetracting(){
        climbState = ClimbState.RETRACT;
    }

    public synchronized void setOff(){
        climbState = ClimbState.OFF;
    }

    public synchronized void setHoming(){
        climbState = ClimbState.HOMING;
        overrideConfig();

    }

    public synchronized void setHighExtending(){
        // /climbSolenoid.set(Value.kForward);
        climbState = ClimbState.HIGH_EXTEND;
    }

    public synchronized void setHighRetracting(){
        climbState = ClimbState.HIGH_RETRACT;
    }

    private void overrideConfig(){
        climbMotor.configFactoryDefault();
    }

    public synchronized void setGains(){
        climbMotor.config_kP(0, SmartDashboard.getNumber("ClimbPID", 0));
    }


}
