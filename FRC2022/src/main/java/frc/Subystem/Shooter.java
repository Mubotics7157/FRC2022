package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    /*final LinearSystem<N1,N1,N1> flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), ShooterConstants.SHOOTER_MOI, 1);
    final KalmanFilter<N1,N1,N1> flywheelObserver = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(3), VecBuilder.fill(.01), .02);
    final LinearQuadraticRegulator<N1,N1,N1> LQR = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(8), VecBuilder.fill(12), .02); // 2nd param is state excursion (rad/s) 3rd param is control effor (Volts)
    final LinearSystemLoop<N1,N1,N1> flywheelLoop = new LinearSystemLoop<>(flywheelPlant, LQR, flywheelObserver, 6, .02);
    */
    public Shooter(){
        flywheel = new TalonFX(ShooterConstants.DEVICE_ID_SHOOTER);
       flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,20);
       flywheel.enableVoltageCompensation(true);
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.voltageCompSaturation = 11;
       config.slot0.kP = .36;
       config.slot0.kD = .036;
       config.slot0.kF = .05;
       flywheel.configAllSettings(config);
       flywheel.setNeutralMode(NeutralMode.Coast);
    }

    public boolean atSpeed(double setpoint){
        lastRPM = setpoint;
        return Math.abs(setpoint - getShooterVelocity()) < ShooterConstants.TOLERANCE_RPM;
    }

    public void rev(double setpointRPM){
    /*
       flywheelLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(setpointRPM)));
       flywheelLoop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor.getEncoder().getVelocity())));
       flywheelLoop.predict(.02);
       shooterMotor.setVoltage(flywheelLoop.getNextR(0));
       SmartDashboard.putNumber("velocity setpoint", setpointRPM);
    */
        flywheel.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(setpointRPM,Constants.ShooterConstants.GEARING));
        SmartDashboard.putNumber("error", setpointRPM-getShooterVelocity());
        SmartDashboard.putNumber("actual RPM", CommonConversions.stepsPerDecisecToRPM(flywheel.getSelectedSensorVelocity(),Constants.ShooterConstants.GEARING));
        SmartDashboard.putNumber("desired RPM", setpointRPM);

    }


    
    private double getShooterVelocity(){
        return CommonConversions.stepsPerDecisecToRPM(flywheel.getSelectedSensorVelocity(),Constants.ShooterConstants.GEARING);
    }

}
