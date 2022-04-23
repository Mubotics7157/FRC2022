package frc.Subsystem;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.util.CommonConversions;

public class Shooter {
    TalonFX flywheelBot;
    TalonFX flywheelTop;
    public Shooter(){
        flywheelBot = new TalonFX(ShooterConstants.DEVICE_ID_BOT_WHEEL);
        flywheelTop = new TalonFX(ShooterConstants.DEVICE_ID_TOP_WHEEL);

        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        topConfig.slot0.kP = .36;
        topConfig.slot0.kD = .045;
        topConfig.slot0.kF = .05;
        flywheelTop.configAllSettings(topConfig);
        TalonFXConfiguration botConfig = new TalonFXConfiguration();

        botConfig.slot0.kP =.36;    
        botConfig.slot0.kD = .045;
        botConfig.slot0.kF =.05;
        flywheelBot.configAllSettings(botConfig);

        flywheelBot.setNeutralMode(NeutralMode.Coast);
        flywheelTop.setNeutralMode(NeutralMode.Coast);

        flywheelBot.overrideLimitSwitchesEnable(false);
        flywheelTop.overrideLimitSwitchesEnable(false);

        flywheelBot.enableVoltageCompensation(true);
        flywheelTop.enableVoltageCompensation(true);

        flywheelTop.configVoltageCompSaturation(10);
        flywheelBot.configVoltageCompSaturation(10);

        flywheelTop.configVelocityMeasurementWindow(1, 1);
        flywheelBot.configVelocityMeasurementWindow(1, 1);
    }
    
    public boolean atSpeed(double topSetpoint, double botSetpoint){
        rev(topSetpoint, botSetpoint);
        SmartDashboard.putNumber("top setpoint", topSetpoint);
        SmartDashboard.putNumber("bot setpoint", botSetpoint);
        boolean atSpeed =  (Math.abs(topSetpoint - getTopRPM()) < ShooterConstants.TOLERANCE_RPM) && (Math.abs(botSetpoint-getBotRPM())<ShooterConstants.TOLERANCE_RPM);
        SmartDashboard.putNumber("actual top", getTopRPM());
        SmartDashboard.putNumber("actual bot", getBotRPM());
        return atSpeed;
    }
    
    public boolean atSpeedRatio(double topSetpoint, double ratio){

        rev(topSetpoint/ratio, topSetpoint);
        return (Math.abs(topSetpoint/ratio - getTopRPM()) < ShooterConstants.TOLERANCE_RPM) && (Math.abs(topSetpoint-getBotRPM())<ShooterConstants.TOLERANCE_RPM);
    }
    public void rev(double topSetpoint, double botSetpoint){
        if(topSetpoint == 0 && botSetpoint == 0){
            flywheelBot.set(ControlMode.PercentOutput,0);
            flywheelTop.set(ControlMode.PercentOutput,0);
        }
        else{
            flywheelBot.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(botSetpoint));
            flywheelTop.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(topSetpoint));
        }

    }

    public void editPorportionalGains(double top, double bot){
        flywheelTop.config_kP(0, top);
        flywheelBot.config_kP(0, bot);
    }

    private double getBotRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelBot.getSelectedSensorVelocity()) ;
    }
    private double getTopRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelTop.getSelectedSensorVelocity());
    }


}
