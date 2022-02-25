package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb {
    private TalonFX climbMotor;

    public Climb(){
        climbMotor = new TalonFX(40);
        //climbMotor.setInverted(InvertType.InvertMotorOutput);
        //climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        /*
         climbMotor.setSelectedSensorPosition(0, 0, 20);

    climbMotor.configPeakOutputForward(1, 20);
    climbMotor.configPeakOutputReverse(-1, 20);
    climbMotor.configNominalOutputForward(0,20);
    climbMotor.configNominalOutputReverse(0,20);

        climbMotor.configFactoryDefault();
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        climbMotor.setInverted(InvertType.InvertMotorOutput);
        climbMotor.configForwardSoftLimitEnable(true);
        climbMotor.configReverseSoftLimitEnable(true);
        climbMotor.configForwardSoftLimitThreshold(1000);
        climbMotor.configReverseSoftLimitThreshold(-1);
        */
    }   

    public void climb(double speed){
        climbMotor.set(ControlMode.PercentOutput, speed);
       // SmartDashboard.putNumber("climb height", climbMotor.getSelectedSensorPosition());
    }
}
