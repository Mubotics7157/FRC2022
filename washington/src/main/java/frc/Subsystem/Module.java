package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;
import frc.util.CommonConversions;

public class Module {
    WPI_TalonFX turnMotor;
    WPI_TalonFX driveMotor;
    WPI_CANCoder absEncoder;

    PIDController turnPID;



       public Module(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
        turnPID = new PIDController(.39, 0, 0); 
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor = new WPI_TalonFX(drivePort,ModuleConstants.SWERVE_CANIVORE_ID);
        turnMotor = new WPI_TalonFX(turnPort,ModuleConstants.SWERVE_CANIVORE_ID);


        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.openloopRamp = ModuleConstants.OPEN_LOOP_RAMP_RATE;
        driveConfig.closedloopRamp = ModuleConstants.CLOSED_LOOP_RAMP_RATE;
        driveMotor.configAllSettings(driveConfig);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,ModuleConstants.TIMEOUT_MS);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(isInverted);
        
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,ModuleConstants.TIMEOUT_MS);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setInverted(false);

        absEncoder = new WPI_CANCoder(encoderPort,ModuleConstants.SWERVE_CANIVORE_ID);
        absEncoder.configFactoryDefault();
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = angleOffset;
        absEncoder.configAllSettings(config);
    }

    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAbsHeading());

            setVelocity(optimizedState.speedMetersPerSecond, .2);
            setTurnRad(optimizedState.angle);
    }

    private void setVelocity(double driveSetpoint, double dt){
        double moduleAccel = (driveSetpoint - getDriveVelocity())/dt;
        double driveFFVolts = ModuleConstants.DRIVE_FEEDFORWARD.calculate(driveSetpoint, moduleAccel);

        if(driveSetpoint==0){
            driveMotor.set(ControlMode.PercentOutput, 0);
        } 
        else 
        driveMotor.set(ControlMode.Velocity, CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint),DemandType.ArbitraryFeedForward,driveFFVolts/RobotController.getBatteryVoltage());
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

    public void updateP(double val){
        turnPID.setP(val);
    }
    public void updateD(double val){
        turnPID.setD(val);
    }

    public void flip(double angle){
        absEncoder.configMagnetOffset(absEncoder.configGetMagnetOffset()+angle);
        
    }
    public void overrideMotors(){
        driveMotor.set(ControlMode.PercentOutput,0);
        turnMotor.set(ControlMode.PercentOutput,0);
    }

}