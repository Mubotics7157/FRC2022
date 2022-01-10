package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.CommonConversions;
import frc.util.SynchronousPID;
import frc.util.Threading.Threaded;

public class Drive extends Threaded {
    WPI_TalonFX leftMaster = new WPI_TalonFX(0);
    WPI_TalonFX leftSlave = new WPI_TalonFX(1);
    WPI_TalonFX rightMaster = new WPI_TalonFX(2);
    WPI_TalonFX rightSlave = new WPI_TalonFX(3);
    private DriveState driveState = DriveState.TELEOP;
    private static Drive instance;


    AnalogGyro m_gyro = new AnalogGyro(1);
    AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

    private SynchronousPID turnPID;


    TalonFXSimCollection leftSimCollection = leftMaster.getSimCollection();
    TalonFXSimCollection rightSimCollection = rightMaster.getSimCollection();
    
    DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(Constants.DriveConstants.kV, Constants.DriveConstants.kA,
                    Constants.DriveConstants.kVAngular, Constants.DriveConstants.kAAngular),
            DCMotor.getFalcon500(2), Constants.PhysicalConstants.GEAR_RATIO,
            Constants.PhysicalConstants.TRACK_WIDTH_METERS, Constants.PhysicalConstants.WHEEL_DIAMETER_METERS, null);

    public Drive(){
        if(Robot.isReal()){
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        motorConfig.neutralDeadband = Constants.DriveConstants.STICK_DEADBAND;
        motorConfig.slot0.kP = Constants.DriveConstants.kP;
        motorConfig.slot0.closedLoopPeakOutput = 1.0;
        motorConfig.closedloopRamp = Constants.DriveConstants.CLOSED_LOOP_RAMP;
        motorConfig.openloopRamp = Constants.DriveConstants.OPEN_LOOP_RAMP;

        leftMaster.configAllSettings(motorConfig);
        leftSlave.configAllSettings(motorConfig);
        rightMaster.configAllSettings(motorConfig);
        rightSlave.configAllSettings(motorConfig);

        rightMaster.overrideLimitSwitchesEnable(false);
        leftMaster.overrideLimitSwitchesEnable(false);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(true);
        rightMaster.setInverted(false);
        }
        
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        leftSlave.setInverted(InvertType.FollowMaster);
        rightSlave.setInverted(InvertType.FollowMaster);

        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave.setNeutralMode(NeutralMode.Coast);

        turnPID = new SynchronousPID(Constants.DriveConstants.TURN_P, Constants.DriveConstants.TURN_I, Constants.DriveConstants.TURN_D);


        zeroEncoders();

        driveSim = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(Constants.DriveConstants.kV, Constants.DriveConstants.kA, Constants.DriveConstants.kVAngular, Constants.DriveConstants.kAAngular),
            DCMotor.getFalcon500(2),
            Constants.PhysicalConstants.GEAR_RATIO,
            Constants.PhysicalConstants.TRACK_WIDTH_METERS,
            Constants.PhysicalConstants.WHEEL_DIAMETER_METERS,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );
    }

    public enum DriveState{
        TELEOP,
        VISION_TRACKING,
        AUTO,
        DONE
    }

    @Override
    public void update() {
        DriveState snapDriveState;
        synchronized(this){
            snapDriveState = driveState;    
        }
        if(Robot.isSimulation())
            simulationPeriodic();
          
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
        tankDriveTeleOp(Robot.operator.getLeftY(), Robot.operator.getLeftY());
    }

    private void tankDriveTeleOp(double leftSpeed, double rightSpeed){
        if(Math.abs(leftSpeed)<=Constants.DriveConstants.STICK_DEADBAND){
            leftSpeed = 0;
        }

        if(Math.abs(rightSpeed)<=Constants.DriveConstants.STICK_DEADBAND){
            rightSpeed = 0;
        }

        tankDrive(leftSpeed, rightSpeed,false,true);
    }

    private void tankDrive(double l, double r, boolean squaredInputs, boolean velocity){
        if(velocity){
            double leftMPS =  l *Constants.DriveConstants.MAX_SPEED_TELE;
            double rightMPS =  r *Constants.DriveConstants.MAX_SPEED_TELE;
            if(squaredInputs){
                leftMPS = Math.copySign(Math.pow(leftMPS, 4), leftMPS);
                rightMPS = Math.copySign(Math.pow(rightMPS, 4), rightMPS);
            }
            DifferentialDriveWheelSpeeds wheelVelocities = new DifferentialDriveWheelSpeeds(leftMPS, rightMPS);
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
        double leftFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        SmartDashboard.putNumber("error", Math.abs(actualLeft-leftSetpoint));

        SmartDashboard.putNumber("voltage", leftFF);

        if(leftSetpoint == 0)
            leftMaster.set(ControlMode.PercentOutput, 0);
        else
            leftMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(leftSetpoint), DemandType.ArbitraryFeedForward,leftFF/12); //divide by 12 bc feedforward returns voltage input to motor, dividing by 12 would make it in range of [-1,1]

        if(rightSetpoint == 0)
            rightMaster.set(ControlMode.PercentOutput, 0);
        else
            rightMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(rightSetpoint), DemandType.ArbitraryFeedForward,rightFF/12);
    }
     /**
   * controls the drivetrain at a given velocity during teleoperated control
   * @param setpoints - desired wheel velocities
   */
    private void tankDriveVelocity(DifferentialDriveWheelSpeeds setpoints){
        double leftSetpoint = setpoints.leftMetersPerSecond;
        double rightSetpoint = setpoints.rightMetersPerSecond;
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
        double leftFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        SmartDashboard.putNumber("error", Math.abs(actualLeft-leftSetpoint));

        SmartDashboard.putNumber("voltage", leftFF);

        if(leftSetpoint == 0)
            leftMaster.set(ControlMode.PercentOutput, 0);
        else
            leftMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(leftSetpoint), DemandType.ArbitraryFeedForward,leftFF/12); //divide by 12 bc feedforward returns voltage input to motor, dividing by 12 would make it in range of [-1,1]

        if(rightSetpoint == 0)
            rightMaster.set(ControlMode.PercentOutput, 0);
        else
            rightMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(rightSetpoint), DemandType.ArbitraryFeedForward,rightFF/12);
    }

     /**
   * controls the drivetrain at a given velocity during teleoperated control
   * @param left left velocity of robot in meters/sec
   * @param right right velocity of robot in meters/sec
   * @param dt delta time
   */
    private void tankDriveVelocity(double leftSetpoint,double rightSetpoint, double dt){
        //get actual velocities of the drivetrain, parameters are setpoints
        double actualLeft = getLeftVelocity();
        double actualRight = getRightVelocity();

        SmartDashboard.putNumber("left Veloctiy", actualLeft);
        SmartDashboard.putNumber("left setpoint", leftSetpoint);

        //find acceleration setpoint using: a = dv/dt
        double leftAccelSetpoint = (leftSetpoint - actualLeft)/dt;
        double rightAccelSetpoint = (rightSetpoint-actualRight)/dt;
        SmartDashboard.putNumber("left accel", leftAccelSetpoint);

        // calculate the voltage needed to get to velocity and acceleration setpoint
        double leftFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        SmartDashboard.putNumber("error", Math.abs(actualLeft-leftSetpoint));

        SmartDashboard.putNumber("voltage", leftFF);

        if(leftSetpoint == 0)
            leftMaster.set(ControlMode.PercentOutput, 0);
        else
            leftMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(leftSetpoint), DemandType.ArbitraryFeedForward,leftFF/12); //divide by 12 bc feedforward returns voltage input to motor, dividing by 12 would make it in range of [-1,1]

        if(rightSetpoint == 0)
            rightMaster.set(ControlMode.PercentOutput, 0);
        else
            rightMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(rightSetpoint), DemandType.ArbitraryFeedForward,rightFF/12);
    }

    private void updateVisionTracking(){

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
        tankDriveVelocity(deltaSpeed*Constants.DriveConstants.MAX_SPEED_TELE,-deltaSpeed*Constants.DriveConstants.MAX_SPEED_TELE);
    }

    private void setChezy(double left, double right, double pivot){
        double l = 0;
        double r = 0;
        if(Math.abs(left)>=Constants.DriveConstants.STICK_DEADBAND)
          l=left;
      
        if(Math.abs(right)>=Constants.DriveConstants.STICK_DEADBAND)
          r = right;

          bootlegCheesyDrive(l, r,pivot);
  }

    private void bootlegCheesyDrive(double throttle, double turn, double pivotTurn){

        if(Math.abs(pivotTurn)>1E-4){
          tankDrive(pivotTurn/2, -pivotTurn/2, false,true);
        }

        else{
        double denominator = Math.sin(Math.PI/2d*.05);

        turn = Math.sin(Math.PI/2d*.05*turn);
        turn = turn / Math.pow(denominator, 2) * Math.abs(throttle);

        double [] speeds = invKinematicsWheelSpeeds(new Twist2d(throttle,0d,turn));
        tankDriveVelocity(speeds[0], speeds[1]);
        }
    }

    private double[] invKinematicsWheelSpeeds(Twist2d velocityVector){
        double l;
        double r;

        if(Math.abs(velocityVector.dtheta)<1E-4){
            l = velocityVector.dx;
            r = velocityVector.dx;
        }

        else{
        double delta_v = (Constants.PhysicalConstants.TRACK_WIDTH_FEET/12) * velocityVector.dtheta / 2d;
        l = velocityVector.dx + delta_v;
        r = velocityVector.dx - delta_v;
        }
        SmartDashboard.putNumber("theta",velocityVector.dtheta);
        SmartDashboard.putNumber("leftVector", l);
        SmartDashboard.putNumber("rightVector", r);


        double scaling = Math.max(1d,Math.max(Math.abs(l), Math.abs(r)));
        double [] speeds = {l/scaling*Constants.DriveConstants.MAX_SPEED_TELE,r/scaling*Constants.DriveConstants.MAX_SPEED_TELE};

        return speeds;
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
        turnPID.setD(turnPID.getD()+kDInc);
    }

    /**
     * @return the velocity of the left side of the drivetrain in meters per second
     * 
     */
    private double getLeftVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity());
    }

    /**
     * @return the velocity of the right side of the drivetrain in meters per second
     * 
     */
    private double getRightVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity());
    }

    public synchronized DifferentialDriveWheelSpeeds getWheelSpeeds(){
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
        return speeds;
    }

    /**
     * 
     * @return distance of the left side of the drivetrain in meters
     */
    public double getLeftDistance(){
        return CommonConversions.stepsToMeters(leftMaster.getSelectedSensorPosition());
    }

    /**
     * 
     * @return distance of the right side of the drivetrain in meters
     */
    public double getRightDistance(){
        return CommonConversions.stepsToMeters(rightMaster.getSelectedSensorPosition());
    }

    public void zeroEncoders(){
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return the heading of the drivetrain as a Rotation2d object
     */
    public synchronized Rotation2d getRotation2d(){
        return m_gyro.getRotation2d();
    }

    /**
     * 
     * @return heading of the drivetrain from [-180,180] degrees
     */
    public double getHeading(){
        return Math.IEEEremainder(m_gyro.getAngle(), 360.0d) * -1.0d; 
    }

    public void resetGyro(){
        m_gyro.reset();
    }

    public void zeroSensors(){
        m_gyro.reset();
        zeroEncoders();
    } 

  public synchronized void simulationPeriodic() {
    driveSim.setInputs(leftMaster.getMotorOutputVoltage(),
                       rightMaster.getMotorOutputVoltage()
    ); 

    driveSim.update(0.02);
    leftSimCollection.setIntegratedSensorRawPosition((int)CommonConversions.metersToSteps(driveSim.getLeftPositionMeters()));
    rightSimCollection.setIntegratedSensorRawPosition((int)CommonConversions.metersToSteps(driveSim.getRightPositionMeters()));
    leftSimCollection.setIntegratedSensorVelocity((int)CommonConversions.metersPerSecToStepsPerDecisec(driveSim.getLeftVelocityMetersPerSecond()));
    rightSimCollection.setIntegratedSensorVelocity((int)CommonConversions.metersPerSecToStepsPerDecisec(driveSim.getRightVelocityMetersPerSecond()));

     m_gyroSim.setAngle(-driveSim.getHeading().getDegrees());

    leftSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
    rightSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
  }
}    

