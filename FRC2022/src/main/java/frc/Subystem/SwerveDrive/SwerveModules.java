package frc.Subystem.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ModuleConstants;
import frc.util.CommonConversions;

public class SwerveModules {
    TalonFX turnMotor;
    TalonFX driveMotor;
    CANCoder absEncoder;
    PIDController turnPID = new PIDController(1, 0, 0); // use internal PID later
    PIDController drivePID = new PIDController(1, 0, 0); // use internal PID later

    double turn=0;
    double drive=0;
    double turnDist=0;

    TalonFXSimCollection turnSimCollection;
    TalonFXSimCollection driveSimCollection;
    CANCoderSimCollection absEncoderSim;
    FlywheelSim turnSim;
    FlywheelSim driveSim;


       public SwerveModules(int drivePort, int turnPort, int turnEncoderPort1){
        turnSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(3.41, .111), DCMotor.getFalcon500(1), 8.16);
        driveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(3.41, .111), DCMotor.getFalcon500(1), 12.8);

        driveMotor = new TalonFX(drivePort);
        turnMotor = new TalonFX(turnPort);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        absEncoder = new CANCoder(turnEncoderPort1);

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        absEncoder.configAllSettings(config);

        absEncoderSim = absEncoder.getSimCollection();
        turnSimCollection = turnMotor.getSimCollection();
        driveSimCollection = driveMotor.getSimCollection();
    }

    public void set(double driveSignal, double angleRad){
        drive = driveSignal;
        driveMotor.set(ControlMode.PercentOutput,drive);
        turn =  turnPID.calculate(getAbsHeading().getRadians(), angleRad);
        turnMotor.set(ControlMode.PercentOutput,turn);
    }

    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = state.optimize(state, getAbsHeading());

        double turnSignal = turnPID.calculate(getAbsHeading().getRadians(), optimizedState.angle.getRadians());
        double driveSignal = drivePID.calculate(getDriveVelocity(), optimizedState.speedMetersPerSecond);
        set(driveSignal,turnSignal);
    }

    private void setVelocity(double driveSetpoint, double turnSetpoint, double dt){
        double actualDriveVel = getDriveVelocity();
        double driveAccelSetpoint = (driveSetpoint-actualDriveVel)/dt;
        double driveFFVolts = ModuleConstants.DRIVE_FEEDFORWARD.calculate(driveSetpoint, driveAccelSetpoint);

        double actualTurnVel = getTurnVelocity();
        double turnAccelSetpoint = (turnSetpoint-actualTurnVel)/dt;
        double turnFFVolts = ModuleConstants.DRIVE_FEEDFORWARD.calculate(turnSetpoint, turnAccelSetpoint);


        if(driveSetpoint==0){
            driveMotor.set(ControlMode.PercentOutput, 0);
        } 
        else 
            driveMotor.set(ControlMode.Velocity, driveSetpoint,DemandType.ArbitraryFeedForward,driveFFVolts/12);
        if(turnSetpoint==0){
            turnMotor.set(ControlMode.PercentOutput, 0);
        }   
        else
            turnMotor.set(ControlMode.Velocity, turnSetpoint,DemandType.ArbitraryFeedForward,turnFFVolts/12);
    }

        public void updateSim(){
        driveSim.setInputVoltage(drive*RobotController.getBatteryVoltage());
        turnSim.setInputVoltage(turn*RobotController.getBatteryVoltage());

        turnSim.update(.02);
        driveSim.update(.02);

        double turnVelStepsPerDecisec = turnSim.getAngularVelocityRPM()*2048/600*12.8;
        double turnSteps = turnVelStepsPerDecisec*10*.02;
        double driveVelStepsPerDecisec = driveSim.getAngularVelocityRPM()*2048/600*6.75;
        double driveSteps = driveVelStepsPerDecisec*10*.02;
        absEncoderSim.setVelocity((int)turnVelStepsPerDecisec);
        absEncoderSim.addPosition((int)turnSteps);
        turnSimCollection.setIntegratedSensorVelocity((int)turnVelStepsPerDecisec);
        turnSimCollection.addIntegratedSensorPosition((int)turnSteps);
        
        driveSimCollection.addIntegratedSensorPosition((int)driveSteps);
        driveSimCollection.setIntegratedSensorVelocity((int)driveVelStepsPerDecisec);

        absEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
        driveSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
        turnSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getAbsHeading()); 
    }

    private Rotation2d getAbsHeading(){
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    private double getDriveVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(driveMotor.getSelectedSensorVelocity());
    }

    private double getTurnVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(turnMotor.getSelectedSensorVelocity());
    }

}
