package frc.Subystem.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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
        double driveSignal = drivePID.calculate(getVelocity(), optimizedState.speedMetersPerSecond);
        set(driveSignal,turnSignal);
    }

        public void updateSim(){
        driveSim.setInputVoltage(drive*RobotController.getBatteryVoltage());
        turnSim.setInputVoltage(turn*RobotController.getBatteryVoltage());
        //driveSim.setInputVoltage(driveMotor.getMotorOutputVoltage());
        //turnSim.setInputVoltage(turnMotor.getMotorOutputVoltage());

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
        return new SwerveModuleState(getVelocity(), getAbsHeading()); //why so slow?
    }

    private Rotation2d getAbsHeading(){
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    private double getVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(driveMotor.getSelectedSensorVelocity());
    }

}
