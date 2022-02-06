package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.util.CommonConversions;

public class Shooter {
    TalonFX flywheel;
    double lastRPM;


    public Shooter(){
        flywheel = new TalonFX(ShooterConstants.DEVICE_ID_SHOOTER);
       flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,20);
       flywheel.enableVoltageCompensation(true);
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.voltageCompSaturation = 11;
       config.slot0.kP = .6;
       config.slot0.kF = .04;
       flywheel.configAllSettings(config);
    }

    public boolean atSpeed(double setpoint){
        lastRPM = setpoint;
        return Math.abs(setpoint - getShooterVelocity()) < ShooterConstants.TOLERANCE_RPM;
    }

    public void rev(double setpointRPM){
        flywheel.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(setpointRPM,Constants.ShooterConstants.GEARING));
        SmartDashboard.putNumber("error", setpointRPM-getShooterVelocity());
        SmartDashboard.putNumber("actual RPM", CommonConversions.stepsPerDecisecToRPM(flywheel.getSelectedSensorVelocity(),Constants.ShooterConstants.GEARING));
        SmartDashboard.putNumber("desired RPM", setpointRPM);

    }


    
    private double getShooterVelocity(){
        return CommonConversions.stepsPerDecisecToRPM(flywheel.getSelectedSensorVelocity(),Constants.ShooterConstants.GEARING);
    }

}
