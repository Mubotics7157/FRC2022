package frc.Subystem.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.util.CommonConversions;

public class SwerveModules {
    TalonFX turnMotor;
    TalonFX driveMotor;
    CANCoder absEncoder;

    double turn = 0;
    double drive = 0;
    double turnDist = 0;

    double velocity = 0;
    double desiredHeading = 0;
    double setpointVel = 0;

    TalonFXSimCollection turnSimCollection;
    TalonFXSimCollection driveSimCollection;
    CANCoderSimCollection absEncoderSim;
    FlywheelSim turnSim;
    FlywheelSim driveSim;
    // sample gains
    // there are ways to move these loops to the talons themselves, but this is a
    // more explicit way to do it when testing
    private final PIDController drivePID = new PIDController(0.01, 0, 0);
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(6.28, 3.14);
    private final ProfiledPIDController turnPID = new ProfiledPIDController(1, 0, 0, rotProfile);

    // FF controllers; I would personally move these to the constants because
    // they're gonna be the same for all modules (i presume)
    // Ks, Kv, Ka need to exist
    // will probably have different values for drive and turn controllers
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV,
            DriveConstants.kA);
    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV,
            DriveConstants.kA);

    public SwerveModules(int drivePort, int turnPort, int turnEncoderPort, double angleOffset, boolean inverted) {
        turnSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(3.41, .111), DCMotor.getFalcon500(1), 8.16);
        driveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(3.41, .111), DCMotor.getFalcon500(1), 12.8);

        driveMotor = new TalonFX(drivePort);
        turnMotor = new TalonFX(turnPort);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setNeutralMode(NeutralMode.Coast);

        turnMotor.enableVoltageCompensation(true);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.openloopRamp = ModuleConstants.OPEN_LOOP_RAMP_RATE;
        driveConfig.closedloopRamp = ModuleConstants.CLOSED_LOOP_RAMP_RATE;
        driveConfig.slot0.kP = .01;

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.voltageCompSaturation = 12;
        turnConfig.supplyCurrLimit.currentLimit = 20;
        turnConfig.supplyCurrLimit.enable = true;
        turnConfig.slot0.kP = .225;
        turnConfig.slot0.kD = 0;
        turnConfig.slot0.kF = 0;
        turnConfig.motionCruiseVelocity = ModuleConstants.MOTION_PROFILE_MAX_SPEED;
        turnConfig.motionAcceleration = ModuleConstants.MOTION_PROFILE_MAX_ACCEL;
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

        // turnMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setNeutralMode(NeutralMode.Coast);
        if (inverted)
            driveMotor.setInverted(InvertType.InvertMotorOutput);

        if (Robot.isSimulation()) {
            absEncoderSim = absEncoder.getSimCollection();
            turnSimCollection = turnMotor.getSimCollection();
            driveSimCollection = driveMotor.getSimCollection();
        }

    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState.optimize(state, getAbsHeading());
        desiredHeading = state.angle.getRadians();
        // unclear as to why this is necessary
        double vel = state.speedMetersPerSecond / DriveConstants.MAX_TANGENTIAL_VELOCITY;
        velocity = state.speedMetersPerSecond;

        double driveOut = drivePID.calculate(driveMotor.getSelectedSensorPosition(), velocity);
        double turnOut = turnPID.calculate(turnMotor.getSelectedSensorPosition(), desiredHeading);
        // calculate the drive and turn motor speeds separately
        // idk if im using the right sensor rate measurement here
        double driveFFOut = driveFF.calculate(state.speedMetersPerSecond);
        double turnFFOut = turnFF.calculate(turnPID.getSetpoint().velocity);

        // set the motor speeds
        // TODO: move this to a talon velocity loop with arbitrary feedforward
        driveMotor.set(ControlMode.PercentOutput, (driveOut + driveFFOut) / 12);
        driveMotor.set(ControlMode.PercentOutput, (turnOut + turnFFOut) / 12);
        // this is really suss
        // else {
        // // setVelocity(optimizedState.speedMetersPerSecond, .2);
        // // driveMotor.set(ControlMode.PercentOutput,vel);
        // // setTurnRad(state.angle);
        // }
    }

    private void setVelocity(double driveSetpoint, double dt) {
        // idk why this method needs to exist
        double actualDriveVel = getDriveVelocity();
        double driveAccelSetpoint = (driveSetpoint - actualDriveVel) / dt;
        // double driveFFVolts =
        // ModuleConstants.DRIVE_FEEDFORWARD.calculate(driveSetpoint,driveAccelSetpoint);

        if (driveSetpoint == 0) {
            driveMotor.set(ControlMode.PercentOutput, 0);
        } else
            driveMotor.set(ControlMode.Velocity, CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint));
        // driveMotor.set(ControlMode.Velocity,
        // CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint),DemandType.ArbitraryFeedForward,driveFFVolts/12);
    }

    private void setTurnRad(Rotation2d turnSetpointRad) {
        double turnSteps = CommonConversions
                .radiansToSteps(turnSetpointRad.getRadians() - getAbsHeading().getRadians());
        double currPosition = turnMotor.getSelectedSensorPosition();
        double setpointSteps = currPosition + turnSteps;

        turnMotor.set(ControlMode.MotionMagic, setpointSteps);
        SmartDashboard.putNumber("turn steps", currPosition);

    }

    public void overrideModule() {
        turnMotor.set(ControlMode.PercentOutput, 0);
        // driveMotor.set(ControlMode.PercentOutput, 0);
    }

    public void updateSim() {
        driveSim.setInputVoltage(drive * RobotController.getBatteryVoltage());
        turnSim.setInputVoltage(turn * RobotController.getBatteryVoltage());

        turnSim.update(.02);
        driveSim.update(.02);

        double turnVelStepsPerDecisec = turnSim.getAngularVelocityRPM() * 2048 / 600 * 12.8;
        double turnSteps = turnVelStepsPerDecisec * 10 * .02;
        double driveVelStepsPerDecisec = driveSim.getAngularVelocityRPM() * 2048 / 600 * 6.75;
        double driveSteps = driveVelStepsPerDecisec * 10 * .02;
        absEncoderSim.setVelocity((int) turnVelStepsPerDecisec);
        absEncoderSim.addPosition((int) turnSteps);
        turnSimCollection.setIntegratedSensorVelocity((int) turnVelStepsPerDecisec);
        turnSimCollection.addIntegratedSensorPosition((int) turnSteps);

        driveSimCollection.addIntegratedSensorPosition((int) driveSteps);
        driveSimCollection.setIntegratedSensorVelocity((int) driveVelStepsPerDecisec);

        absEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
        driveSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
        turnSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsHeading());
    }

    private Rotation2d getAbsHeading() {
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    public double getDriveVelocity() {
        return CommonConversions.stepsPerDecisecToMetersPerSec(driveMotor.getSelectedSensorVelocity());
    }

    public void testMotors(double drive, double turn) {
        driveMotor.set(ControlMode.PercentOutput, drive);
        turnMotor.set(ControlMode.PercentOutput, turn);
    }

    private void resetEncoders() {
        turnMotor.setSelectedSensorPosition(CommonConversions.radiansToSteps(getAbsHeading().getRadians()));
    }

    public double getTurn() {
        return desiredHeading;
    }

    public void invertMotor() {
        turnMotor.setInverted(true);
    }

    public double getVelocity() {
        return velocity;
    }

}
