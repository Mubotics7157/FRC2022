package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb {
    private TalonFX climbMotor;

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
        config.slot0.kP = .5;
        climbMotor.configAllSettings(config);
    }   

    public void climb(double speed){
        climbMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("climb height", climbMotor.getSelectedSensorPosition());
    }

    public void goUp(){
        climbMotor.set(ControlMode.Position,77500);
    }

    public void goDown(){
        climbMotor.set(ControlMode.Position,500);
    }
}
