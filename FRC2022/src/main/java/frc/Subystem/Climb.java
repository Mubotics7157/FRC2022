package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Climb {
    private TalonFX climbMotor;

    public Climb(){
        climbMotor = new TalonFX(0);

        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }   

    public void climb(double speed){
        climbMotor.set(ControlMode.PercentOutput, speed);
    }
}
