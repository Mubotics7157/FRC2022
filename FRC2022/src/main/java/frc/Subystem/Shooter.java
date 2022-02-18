package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.util.CommonConversions;

public class Shooter {
    TalonFX flywheelBot;
    TalonFX flywheelTop;
    private double bottomSpeed=.3;
    private double topSpeed = .2;
    /*final LinearSystem<N1,N1,N1> flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), ShooterConstants.SHOOTER_MOI, 1);
    final KalmanFilter<N1,N1,N1> flywheelObserver = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(3), VecBuilder.fill(.01), .02);
    final LinearQuadraticRegulator<N1,N1,N1> LQR = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(8), VecBuilder.fill(12), .02); // 2nd param is state excursion (rad/s) 3rd param is control effor (Volts)
    final LinearSystemLoop<N1,N1,N1> flywheelLoop = new LinearSystemLoop<>(flywheelPlant, LQR, flywheelObserver, 6, .02);
    */
    public Shooter(){
        flywheelBot = new TalonFX(1);
        flywheelTop = new TalonFX(0);
        //TalonFXConfiguration config = new TalonFXConfiguration();
        //config.voltageCompSaturation = 10;
        //config.slot0.kP = .36;
        //config.slot0.kD = .036;
        //config.slot0.kF = .05;
        //flywheelBot.configAllSettings(config);
        //flywheelBot.configAllSettings(config);
        //flywheelTop.setNeutralMode(NeutralMode.Brake);
        //flywheelTop.setNeutralMode(NeutralMode.Brake);
        //flywheelBot.setInverted(true);
        //flywheelTop.setInverted(InvertType.InvertMotorOutput);
        flywheelBot.configFactoryDefault();
        flywheelBot.setInverted(InvertType.InvertMotorOutput);
    }

    public boolean atSpeed(double topSetpoint, double botSetpoint){
        rev(topSetpoint, botSetpoint);
        SmartDashboard.putNumber("Top RPM", topSetpoint);
        SmartDashboard.putNumber("Bot RPM", botSetpoint);
        return (Math.abs(topSetpoint - getTopRPM()) < ShooterConstants.TOLERANCE_RPM) && (Math.abs(botSetpoint-getBotRPM())<ShooterConstants.TOLERANCE_RPM);
    }
    
    public boolean atSpeedRatio(double topSetpoint, double ratio){

        rev(topSetpoint/ratio, topSetpoint);
        SmartDashboard.putNumber("Top RPM", topSetpoint);
        return (Math.abs(topSetpoint/ratio - getTopRPM()) < ShooterConstants.TOLERANCE_RPM) && (Math.abs(topSetpoint-getBotRPM())<ShooterConstants.TOLERANCE_RPM);
    }
    public void rev(double topSetpoint, double botSetpoint){
        flywheelBot.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(botSetpoint,1));
        flywheelTop.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(topSetpoint,1));
        SmartDashboard.putNumber("error", topSetpoint-getTopRPM());
        SmartDashboard.putNumber("actual RPM", CommonConversions.stepsPerDecisecToRPM(flywheelTop.getSelectedSensorVelocity(),1));
        SmartDashboard.putNumber("desired RPM", topSetpoint);

    }

    public void adjustShooterSpeeds(double topAdjust, double botAdjust){
        topSpeed += topAdjust; bottomSpeed += botAdjust;
    }

    public void setShooter(double top, double bot){
      flywheelTop.set(ControlMode.PercentOutput, top);
      flywheelBot.set(ControlMode.PercentOutput, bot); 
        
    }


    private double getBotRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelBot.getSelectedSensorVelocity(), 1);
    }
    private double getTopRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelTop.getSelectedSensorVelocity(), 1);
    }

}
