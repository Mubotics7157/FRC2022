package frc.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.CommonConversions;
import frc.util.SynchronousPID;
import frc.util.Threading.Threaded;


public class Drive extends Threaded {
    private TalonFX leftMaster;
    private TalonFX leftSlave;
    private TalonFX rightMaster;
    private TalonFX rightSlave;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private Rotation2d desiredHeading;
    private DriveState driveState = DriveState.TELEOP;
    private static Drive instance;


    private SynchronousPID turnPID;
    private SynchronousPID forwardPID;

    DifferentialDrivetrainSim driveSim;

    public Drive(){
        leftMaster = new TalonFX(Constants.DiffDriveConstants.DEVICE_ID_LEFT_SLAVE);
        leftSlave = new TalonFX(Constants.DiffDriveConstants.DEVICE_ID_LEFT_SLAVE);
        rightMaster = new TalonFX(Constants.DiffDriveConstants.DEVICE_ID_RIGHT_MASTER);
        rightSlave = new TalonFX(Constants.DiffDriveConstants.DEVICE_ID_RIGHT_SLAVE);
        

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        motorConfig.neutralDeadband = Constants.DiffDriveConstants.STICK_DEADBAND;
        motorConfig.slot0.kP = Constants.DiffDriveConstants.kP;
        motorConfig.slot0.closedLoopPeakOutput = 1.0;
        motorConfig.closedloopRamp = Constants.DiffDriveConstants.CLOSED_LOOP_RAMP;
        motorConfig.openloopRamp = Constants.DiffDriveConstants.OPEN_LOOP_RAMP;

        leftMaster.configAllSettings(motorConfig);
        //leftSlave.configAllSettings(motorConfig);
        rightMaster.configAllSettings(motorConfig);
        rightSlave.configAllSettings(motorConfig);

        rightMaster.overrideLimitSwitchesEnable(false);
        leftMaster.overrideLimitSwitchesEnable(false);

        //leftSlave.follow(leftMaster);
        //rightSlave.follow(rightMaster);

        leftMaster.setInverted(true);
        rightMaster.setInverted(false);
        //leftSlave.setInverted(InvertType.FollowMaster);
        rightSlave.setInverted(InvertType.FollowMaster);

        leftMaster.setNeutralMode(NeutralMode.Coast);
        //leftSlave.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave.setNeutralMode(NeutralMode.Coast);

        turnPID = new SynchronousPID(-.015, 0, .007,0);//target tracking PID
        //turnPID = new SynchronousPID(-.008, 0, 0,0);//cargo tracking


        zeroEncoders();

        );


    }

    public enum DriveState{
        TELEOP,
        VISION_TRACKING,
    }
    @Override
    public void update() {
        DriveState snapDriveState;
        synchronized(this){
            snapDriveState = driveState;    
        }
        switch(snapDriveState){
            case TELEOP:
                updateTeleop();
                SmartDashboard.putString("Drive State", "teleop");
                break;
            case VISION_TRACKING:
                updateVisionTracking();
                SmartDashboard.putString("Drive State", "vision tracking");
                break;
        }
        
    }

    public static Drive getInstance(){
        if(instance==null)
            instance = new Drive();
        return instance;
    }

    public void setTeleop(){
        synchronized(this){
        driveState = DriveState.TELEOP;
        }
    }



    public synchronized void setTracking(){
        driveState = DriveState.VISION_TRACKING;
    }

    private void updateTeleop(){
        tankDriveTeleOp(Robot.operator.getRawAxis(5), Robot.operator.getRawAxis(1));
    }


    private void tankDriveTeleOp(double leftSpeed, double rightSpeed){
        if(Math.abs(leftSpeed)<=Constants.DiffDriveConstants.STICK_DEADBAND){
            leftSpeed = 0;
        }

        if(Math.abs(rightSpeed)<=Constants.DiffDriveConstants.STICK_DEADBAND){
            rightSpeed = 0;
        }

        tankDrive(leftSpeed, rightSpeed,false,true);
    }

    private void tankDrive(double l, double r, boolean squaredInputs, boolean velocity){
        if(velocity){
            double leftMPS =  l *Constants.DiffDriveConstants.MAX_SPEED_TELE;
            double rightMPS =  r *Constants.DiffDriveConstants.MAX_SPEED_TELE;
            if(squaredInputs){
                leftMPS = Math.copySign(Math.pow(leftMPS, 4), leftMPS);
                rightMPS = Math.copySign(Math.pow(rightMPS, 4), rightMPS);
            }
            tankDriveVelocity(leftMPS, rightMPS);
        }
    
        else{
            if(squaredInputs){
                l = Math.copySign(Math.pow(l, 4), l);
                r = Math.copySign(Math.pow(r, 4), r);
                }
            setRawSpeeds(l, r);
        }
    }

 /**
   * controls the drivetrain at a given velocity during teleoperated control
   * @param left left velocity of robot in meters/sec
   * @param right right velocity of robot in meters/sec
   */
    private void tankDriveVelocity(double leftSetpoint,double rightSetpoint){
        //get actual velocities of the drivetrain, parameters are setpoints
        double actualLeft = getLeftVelocity();
        double actualRight = getRightVelocity();

        SmartDashboard.putNumber("left Veloctiy", actualLeft);
        SmartDashboard.putNumber("left setpoint", leftSetpoint);

        //find acceleration setpoint using: a = dv/dt
        //in this case dt is .2 because this runs every 20ms
        double leftAccelSetpoint = (leftSetpoint - actualLeft)/.2;
        double rightAccelSetpoint = (rightSetpoint-actualRight)/.2;
        SmartDashboard.putNumber("left accel", leftAccelSetpoint);

        // calculate the voltage needed to get to velocity and acceleration setpoint
        double leftFF = Constants.DiffDriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DiffDriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        SmartDashboard.putNumber("error", Math.abs(actualLeft-leftSetpoint));

        SmartDashboard.putNumber("voltage", leftFF);

        if(leftSetpoint == 0)
            leftMaster.set(ControlMode.PercentOutput, 0);
        else
            leftMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(leftSetpoint,Constants.DiffDriveConstants.GEAR_RATIO), DemandType.ArbitraryFeedForward,leftFF/12); //divide by 12 bc feedforward returns voltage input to motor, dividing by 12 would make it in range of [-1,1]

        if(rightSetpoint == 0)
            rightMaster.set(ControlMode.PercentOutput, 0);
        else
            rightMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(rightSetpoint,Constants.DiffDriveConstants.GEAR_RATIO), DemandType.ArbitraryFeedForward,rightFF/12);
        
    }

    /**
   * controls the drivetrain at a given velocity during teleoperated control
   * @param left left velocity of robot in meters/sec
   * @param right right velocity of robot in meters/sec
   * @param dt change in time
   */
    private void tankDriveVelocity(double leftSetpoint,double rightSetpoint,double dt){
        //get actual velocities of the drivetrain, parameters are setpoints
        double actualLeft = getLeftVelocity();
        double actualRight = getRightVelocity();

        SmartDashboard.putNumber("left Veloctiy", actualLeft);
        SmartDashboard.putNumber("left setpoint", leftSetpoint);

        //find acceleration setpoint using: a = dv/dt
        //in this case dt is .2 because this runs every 20ms
        double leftAccelSetpoint = (leftSetpoint - actualLeft)/dt;
        double rightAccelSetpoint = (rightSetpoint-actualRight)/dt;

        // calculate the voltage needed to get to velocity and acceleration setpoint
        double leftFF = Constants.DiffDriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DiffDriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        SmartDashboard.putNumber("error", Math.abs(actualLeft-leftSetpoint));

        if(leftSetpoint == 0)
            leftMaster.set(ControlMode.PercentOutput, 0);
        else
            leftMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(leftSetpoint,Constants.DiffDriveConstants.GEAR_RATIO), DemandType.ArbitraryFeedForward,leftFF/12); //divide by 12 bc feedforward returns voltage input to motor, dividing by 12 would make it in range of [-1,1]

        if(rightSetpoint == 0)
            rightMaster.set(ControlMode.PercentOutput, 0);
        else
            rightMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(rightSetpoint,Constants.DiffDriveConstants.GEAR_RATIO), DemandType.ArbitraryFeedForward,rightFF/12);
    }

    private void updateVisionTracking(){
        //double deltaSpeed = turnPID.calculate(-VisionManager.getInstance().getTargetYaw(), 0);
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getYawRotation2d()).unaryMinus().getDegrees();
        SmartDashboard.putNumber("Yaw error", error);
        double deltaSpeed = turnPID.update(error);
     SmartDashboard.putNumber("deltaSpeed", deltaSpeed);

		if (Math.abs(error) < 10 && deltaSpeed < 0.2) {
			tankDriveVelocity(0, 0);
			synchronized (this) {
				driveState = DriveState.TELEOP;
          } 
        }
        tankDriveVelocity(deltaSpeed*Constants.DiffDriveConstants.MAX_SPEED_TELE,-deltaSpeed*Constants.DiffDriveConstants.MAX_SPEED_TELE);
    }

    public boolean isFinished(){
        return driveState == DriveState.DONE;
    }

    /**
     * 
     * @param leftSpeed percent output of left motors on [-1,1]
     * @param rightSpeed percent output of right motors on [-1,1]
     */
    public void setRawSpeeds(double leftSpeed, double rightSpeed){
        leftMaster.set(ControlMode.PercentOutput, leftSpeed);
        rightMaster.set(ControlMode.PercentOutput, rightSpeed);
    }
    
    public synchronized void editPIDGains(double kPInc, double kIInc, double kDInc){
        turnPID.setP(turnPID.getP()+kPInc);
        turnPID.setI(turnPID.getI()+kIInc);
        turnPID.setD(turnPID.getD()*kDInc);
        SmartDashboard.putNumber("Proportional", turnPID.getP());
        SmartDashboard.putNumber("Derrivative", turnPID.getD());
    }

    /**
     * @return the velocity of the left side of the drivetrain in meters per second
     * 
     */
    private double getLeftVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity(),Constants.DiffDriveConstants.GEAR_RATIO);
    }

    /**
     * @return the velocity of the right side of the drivetrain in meters per second
     * 
     */
    private double getRightVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity(),Constants.DiffDriveConstants.GEAR_RATIO);
    }

    /**
     * 
     * @return distance of the left side of the drivetrain in meters
     */
    public double getLeftDistance(){
        return CommonConversions.stepsToMeters(leftMaster.getSelectedSensorPosition(),Constants.DiffDriveConstants.GEAR_RATIO);
    }

    /**
     * 
     * @return distance of the right side of the drivetrain in meters
     */
    public double getRightDistance(){
        return CommonConversions.stepsToMeters(rightMaster.getSelectedSensorPosition(),Constants.DiffDriveConstants.GEAR_RATIO);
    }

    public void zeroEncoders(){
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return the heading of the drivetrain as a Rotation2d object
     */
    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }

    /**
     * 
     * @return heading of the drivetrain from [-180,180] degrees
     */
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d; 
    }

    public void resetGyro(){
        gyro.reset();
    }

    public void zeroSensors(){
        gyro.zeroYaw();
        zeroEncoders();
    } 
}