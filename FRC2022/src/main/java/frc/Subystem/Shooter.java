package frc.Subystem;

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
        flywheelBot = new TalonFX(20);
        flywheelTop = new TalonFX(19);
        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        topConfig.voltageCompSaturation = 10;
        topConfig.slot0.kP = .22;
        topConfig.slot0.kD = .002;
        topConfig.slot0.kF = .05;
        flywheelTop.configAllSettings(topConfig);
        TalonFXConfiguration botConfig = new TalonFXConfiguration();
        botConfig.voltageCompSaturation = 10;
        botConfig.slot0.kP =.22;
        botConfig.slot0.kF =.05;
        flywheelBot.configAllSettings(botConfig);
        flywheelBot.setNeutralMode(NeutralMode.Coast);
        flywheelTop.setNeutralMode(NeutralMode.Coast);
        flywheelBot.overrideLimitSwitchesEnable(false);
        flywheelTop.overrideLimitSwitchesEnable(false);
    }

    public boolean atSpeed(double topSetpoint, double botSetpoint){
        rev(topSetpoint, botSetpoint);
        SmartDashboard. putNumber("Top RPM", topSetpoint);
        SmartDashboard.putNumber("Bot RPM", botSetpoint);
        boolean atSpeed =  (Math.abs(topSetpoint - getTopRPM()) < ShooterConstants.TOLERANCE_RPM) && (Math.abs(botSetpoint-getBotRPM())<ShooterConstants.TOLERANCE_RPM);
        SmartDashboard.putBoolean("at speed", atSpeed);
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
        SmartDashboard.putNumber("error", botSetpoint-getBotRPM());

    }

    private double getBotRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelBot.getSelectedSensorVelocity()) ;
    }
    private double getTopRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelTop.getSelectedSensorVelocity());
    }

}
