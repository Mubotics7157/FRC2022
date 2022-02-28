package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Threading.Threaded;

public class Climb extends Threaded {
    private TalonFX climbMotor;
    private ClimbState climbState = ClimbState.OFF;

    private static Climb instance;

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

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.forwardSoftLimitEnable =  true;
        config.reverseSoftLimitEnable = true;
        config.forwardSoftLimitThreshold = 776071;
        config.reverseSoftLimitThreshold = -1;
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
                climb(.8);
                break;
            case RETRACT:
                SmartDashboard.putString("climb state", "Retracting");
                climb(-.5);
                //goDown();
                break;
            case HOMING:
                SmartDashboard.putString("climb state", "Homing");
                climb(Robot.driver.getRightY());
                break;
            case OFF:
                SmartDashboard.putString("climb state", "Off");
                climb(0);
                break;
        }
    }

    private void climb(double speed){
        climbMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("climb height", climbMotor.getSelectedSensorPosition());
    }

    private void goUp(){
        climbMotor.set(ControlMode.Position  ,775000);
    }

    private void goDown(){
        climbMotor.set(ControlMode.Position,500);
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


    private void overrideConfig(){
        climbMotor.configFactoryDefault();
    }


}
