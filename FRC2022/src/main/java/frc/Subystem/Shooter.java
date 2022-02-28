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
    private double bottomSpeed=-.4;
    private double topSpeed = .3;
    /*final LinearSystem<N1,N1,N1> flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), ShooterConstants.SHOOTER_MOI, 1);
    final KalmanFilter<N1,N1,N1> flywheelObserver = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(3), VecBuilder.fill(.01), .02);
    final LinearQuadraticRegulator<N1,N1,N1> LQR = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(8), VecBuilder.fill(12), .02); // 2nd param is state excursion (rad/s) 3rd param is control effor (Volts)
    final LinearSystemLoop<N1,N1,N1> flywheelLoop = new LinearSystemLoop<>(flywheelPlant, LQR, flywheelObserver, 6, .02);
    */
    public Shooter(){
        flywheelBot = new TalonFX(20);
        flywheelTop = new TalonFX(19);
        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        topConfig.voltageCompSaturation = 10;
        topConfig.slot0.kP = .22;
        topConfig.slot0.kI = .00002;
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

    public void adjustShooterSpeeds(double topAdjust, double botAdjust){
        topSpeed += topAdjust; bottomSpeed += botAdjust;
    }

    public void setShooter(){
      flywheelTop.set(ControlMode.PercentOutput, topSpeed);
      flywheelBot.set(ControlMode.PercentOutput, bottomSpeed); 
      SmartDashboard.putNumber("top", topSpeed);
      SmartDashboard.putNumber("bot", bottomSpeed);
        
    }


    private double getBotRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelBot.getSelectedSensorVelocity()) ;
    }
    private double getTopRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelTop.getSelectedSensorVelocity());
    }

}
