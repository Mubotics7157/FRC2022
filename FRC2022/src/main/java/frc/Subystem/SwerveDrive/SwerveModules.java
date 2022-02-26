package frc.Subystem.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.util.CommonConversions;

public class SwerveModules {
    TalonFX turnMotor;
    TalonFX driveMotor;
    CANCoder absEncoder;

    PIDController turnPID;


       public SwerveModules(int drivePort, int turnPort, int turnEncoderPort, double angleOffset, double pushinP){
        turnPID = new PIDController(pushinP, 0, 0);

        driveMotor = new TalonFX(drivePort);
        turnMotor = new TalonFX(turnPort);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setNeutralMode(NeutralMode.Coast);

        turnMotor.enableVoltageCompensation(true);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,20);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.openloopRamp = ModuleConstants.OPEN_LOOP_RAMP_RATE;
        driveConfig.closedloopRamp = ModuleConstants.CLOSED_LOOP_RAMP_RATE;

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.openloopRamp = ModuleConstants.OPEN_LOOP_RAMP_RATE;
        turnConfig.closedloopRamp = ModuleConstants.CLOSED_LOOP_RAMP_RATE;

        turnMotor.configAllSettings(turnConfig);
        driveMotor.configAllSettings(driveConfig);
        absEncoder = new CANCoder(turnEncoderPort);

        absEncoder.configFactoryDefault();
        
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = angleOffset;
        absEncoder.configAllSettings(config);

    }

    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = state.optimize(state, getAbsHeading());

        if(Math.abs(optimizedState.speedMetersPerSecond) < .1){
           turnMotor.set(ControlMode.PercentOutput,0);
           driveMotor.set(ControlMode.PercentOutput, 0);
        }
        else{
            setVelocity(optimizedState.speedMetersPerSecond, .2);
            setTurnRad(optimizedState.angle);
        }
    }

    private void setVelocity(double driveSetpoint, double dt){
        double actualDriveVel = getDriveVelocity();
        double driveAccelSetpoint = (driveSetpoint-actualDriveVel)/dt;
        double driveFFVolts = ModuleConstants.DRIVE_FEEDFORWARD.calculate(driveSetpoint,driveAccelSetpoint);

        if(driveSetpoint==0){
            driveMotor.set(ControlMode.PercentOutput, 0);
        } 
        else 
        //driveMotor.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint));
        driveMotor.set(ControlMode.Velocity, CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint),DemandType.ArbitraryFeedForward,driveFFVolts/12);
    }


    private void setTurnRad(Rotation2d turnSetpointRad){
        double output = turnPID.calculate(getAbsHeading().getRadians(), turnSetpointRad.getRadians());

        turnMotor.set(ControlMode.PercentOutput,output);
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getAbsHeading()); 
    }

    private Rotation2d getAbsHeading(){
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    public double getDriveVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(driveMotor.getSelectedSensorVelocity());
    }

}
