package frc.Subystem.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.util.CommonConversions;

public class SwerveModules {
    TalonFX turnMotor;
    TalonFX driveMotor;
    CANCoder absEncoder;

    double turn=0;
    double drive=0; 
    double turnDist=0;

    double velocity = 0;
    double error = 0;
    double setpointVel = 0;

    TalonFXSimCollection turnSimCollection;
    TalonFXSimCollection driveSimCollection;
    CANCoderSimCollection absEncoderSim;
    FlywheelSim turnSim;
    FlywheelSim driveSim;


       public SwerveModules(int drivePort, int turnPort, int turnEncoderPort, double angleOffset,boolean inverted){
        turnSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(3.41, .111), DCMotor.getFalcon500(1), 8.16);
        driveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(3.41, .111), DCMotor.getFalcon500(1), 12.8);

        driveMotor = new TalonFX(drivePort);
        turnMotor = new TalonFX(turnPort);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setNeutralMode(NeutralMode.Coast);

        turnMotor.enableVoltageCompensation(true);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,20);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.openloopRamp = ModuleConstants.OPEN_LOOP_RAMP_RATE;
        driveConfig.closedloopRamp = ModuleConstants.CLOSED_LOOP_RAMP_RATE;

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.voltageCompSaturation = 12;
        turnConfig.supplyCurrLimit.currentLimit = 20;
        turnConfig.supplyCurrLimit.enable = true;
        turnConfig.slot0.kP = .05 ;
        turnConfig.slot0.kD = 0;
        turnConfig.slot0.kF = 0;
        turnConfig.motionCruiseVelocity = ModuleConstants.MOTION_PROFILE_MAX_SPEED;
        turnConfig.motionAcceleration = ModuleConstants.MOTION_PROFILE_MAX_ACCEL;
        turnConfig.openloopRamp = ModuleConstants.OPEN_LOOP_RAMP_RATE;
        turnConfig.closedloopRamp = ModuleConstants.CLOSED_LOOP_RAMP_RATE;

        turnMotor.configAllSettings(turnConfig);
        driveMotor.configAllSettings(driveConfig);
        absEncoder = new CANCoder(turnEncoderPort);

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = angleOffset;
        absEncoder.configAllSettings(config);

        //turnMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setNeutralMode(NeutralMode.Coast);
        if(inverted)
            driveMotor.setInverted(InvertType.InvertMotorOutput);


        if(Robot.isSimulation()){
        absEncoderSim = absEncoder.getSimCollection();
        turnSimCollection = turnMotor.getSimCollection();
        driveSimCollection = driveMotor.getSimCollection();
        }

        
    }


    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = state.optimize(state, getAbsHeading());
        error = optimizedState.angle.getDegrees();
        double vel = optimizedState.speedMetersPerSecond/DriveConstants.MAX_TANGENTIAL_VELOCITY;
        velocity = optimizedState.speedMetersPerSecond;

        if(Math.abs(optimizedState.speedMetersPerSecond) < .1){
           turnMotor.set(ControlMode.PercentOutput,0);
           driveMotor.set(ControlMode.PercentOutput, 0);
        }
        else{
            //setVelocity(optimizedState.speedMetersPerSecond/10, .2);
            driveMotor.set(ControlMode.PercentOutput,vel);
            setTurnRad(optimizedState.angle);
        }
    }

    private void setVelocity(double driveSetpoint, double dt){
        double actualDriveVel = getDriveVelocity();
        double driveAccelSetpoint = (driveSetpoint-actualDriveVel)/dt;
        double driveFFVolts = ModuleConstants.DRIVE_FEEDFORWARD.calculate(driveSetpoint, driveAccelSetpoint);

        if(driveSetpoint==0){
            driveMotor.set(ControlMode.PercentOutput, 0);
        } 
        else 
            driveMotor.set(ControlMode.Velocity, driveSetpoint,DemandType.ArbitraryFeedForward,driveFFVolts/12);
    }


    private void setTurnRad(Rotation2d turnSetpointRad){
        double turnSteps = CommonConversions.radiansToSteps(turnSetpointRad.getRadians()-getAbsHeading().getRadians());
        double currPosition = turnMotor.getSelectedSensorPosition();
        double setpointSteps = currPosition+turnSteps;

        turnMotor.set(ControlMode.MotionMagic,setpointSteps);
    }

    public void overrideModule(){
        turnMotor.set(ControlMode.PercentOutput, 0);
        //driveMotor.set(ControlMode.PercentOutput, 0);
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

    public double getDriveVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(driveMotor.getSelectedSensorVelocity());
    }

    public void testMotors(double drive, double turn){
            driveMotor.set(ControlMode.PercentOutput, drive);
            turnMotor.set(ControlMode.PercentOutput, turn);
    }

    private void resetEncoders(){
        turnMotor.setSelectedSensorPosition(CommonConversions.radiansToSteps(getAbsHeading().getRadians()));
    }

    public double getTurn(){
    return error;
    }

    public void invertMotor(){
        turnMotor.setInverted(true);
    }

    public double getVelocity(){
        return velocity;
    }


}
