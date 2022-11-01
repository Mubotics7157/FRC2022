package frc.Subsystem;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.util.AbstractSubsystem;

public class Drive extends AbstractSubsystem{
    
    public enum DriveState{
        TELE,
        VISION,
        LOCKED,
        ODOM_ALIGN,
        AUTO,
        DONE
    }

    private static DriveState driveState = DriveState.TELE;
    private static final Drive driveInstance = new Drive();
    

    private Module frontLeft = DriveConstants.FRONT_LEFT_MODULE;
    private Module frontRight = DriveConstants.FRONT_RIGHT_MODULE;
    private Module rearRight = DriveConstants.REAR_RIGHT_MODULE;
    private Module rearLeft = DriveConstants.REAR_LEFT_MODULE;

    WPI_Pigeon2 gyro =new WPI_Pigeon2(30, ModuleConstants.SWERVE_CANIVORE_ID);



    TrapezoidProfile.Constraints visionRotProfile = new TrapezoidProfile.Constraints(2*Math.PI,8*Math.PI);
    ProfiledPIDController visionRotController = new ProfiledPIDController(DriveConstants.TURN_kP, 0, DriveConstants.TURN_kD,visionRotProfile);

    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI,Math.PI);
    ProfiledPIDController rotController = new ProfiledPIDController(-4, 0, 0,rotProfile);

    PIDController xController = new PIDController(DriveConstants.AUTO_CONTROLLER_kP, 0, 0);
    PIDController yController = new PIDController(DriveConstants.AUTO_CONTROLLER_kP, 0, 0);

    HolonomicDriveController autoController; 

    final Lock currentAutoTrajectoryLock = new ReentrantLock();
    Trajectory currTrajectory;
    private volatile Rotation2d desiredAutoHeading;
    private double autoStartTime;
    double maxAngVel = 2 * Math.PI;

        
    private Drive() {
        super(20,20);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        //rotController.enableContinuousInput(-180, 180);
        autoController = new HolonomicDriveController(xController, yController, rotController); 
        autoController.setTolerance(new Pose2d(.5,.5,Rotation2d.fromDegrees(10)));
        visionRotController.enableContinuousInput(-Math.PI, Math.PI);
        visionRotController.setTolerance(Units.degreesToRadians(2));
    }
    
    public static Drive getInstance(){
        return driveInstance;
    }
    
    @Override
    public void update() {
        DriveState snapDriveState;
        synchronized(this){
            snapDriveState = driveState;
        }
        switch(snapDriveState){
            case TELE:
                updateManual();
                break;
            case VISION:
                updateAlign();
                break;
            case ODOM_ALIGN:
                updateOdomAlign();
                break;
            case LOCKED:
                updateLocked();
                break;
            case AUTO:
                updateAuto();
                break;
            case DONE:
                stopMotors();
                break;
        }
    }

    private void updateManual(){
        double str = Robot.driver.getLeftX();
        double fwd = Robot.driver.getLeftY();
        double rot = Robot.driver.getRightX();

        if(Math.abs(str) < .1)
            str = 0;
        if(Math.abs(fwd) < .1)
            fwd = 0;
        if(Math.abs(rot) < .15)
            rot = 0;

        driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
        str*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
        rot*maxAngVel,
        getDriveHeading()));

    }

    private void updateLocked(){
        frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        //setModuleStates(DriveConstants.LOCKED_MODULE_STATES);
        if(Math.abs(Robot.driver.getLeftX()) > .1 || Math.abs(Robot.driver.getLeftY())>.1 || Math.abs(Robot.driver.getRightX())>.15)
            setDriveState(DriveState.TELE);

    }

    private void updateManual(boolean fieldOriented, double rotModifier){
        double str = Robot.driver.getLeftX();
        double fwd = Robot.driver.getLeftY();
        double rot = rotModifier;

        if(Math.abs(str) < .1)
            str = 0;
        if(Math.abs(fwd) < .1)
            fwd = 0;
        if(fieldOriented)
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
            str*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
            rot*(2*Math.PI),
            Odometry.getInstance().getOdometry().getRotation()));
    }

    private void updateAlign(){
        if(VisionManager.getInstance().hasVisionTarget()){
        Rotation2d onTarget = Rotation2d.fromDegrees(3);
        double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).getRadians();


        if(Math.abs(error)<Units.degreesToRadians(3))
            error = 0;
        double deltaSpeed = visionRotController.calculate(error);
           if(visionRotController.atGoal())
            setDriveState(DriveState.LOCKED);
            
        updateManual(true,deltaSpeed);
        }
        else{
           setDriveState(DriveState.TELE);
        }
    }

    private void updateOdomAlign(){
        Rotation2d onTarget = Rotation2d.fromDegrees(3);
        double error;
        if(VisionManager.getInstance().hasVisionTarget())
            error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).getRadians();
        else
            error = onTarget.rotateBy(Odometry.getInstance().getAngleToTarget()).getRadians();


        if(Math.abs(error)<Units.degreesToRadians(3))
            error = 0;
        double deltaSpeed = visionRotController.calculate(error);
           if(visionRotController.atGoal())
            setDriveState(DriveState.TELE);
            
        updateManual(true,deltaSpeed);
        }


    private void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(getModuleStates(), DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);
    }

    private void updateAuto(){
        currentAutoTrajectoryLock.lock();

        try{
            Trajectory.State goal = currTrajectory.sample(getAutoTime());
            Rotation2d target = desiredAutoHeading;
            SmartDashboard.putNumber("desired rotation", target.getDegrees());
            SmartDashboard.putNumber("heading error",Math.abs(target.rotateBy(Odometry.getInstance().getOdometry().getRotation()).getDegrees()));
            SmartDashboard.putNumber("time elapsed", getAutoTime());

            ChassisSpeeds desiredSpeeds = autoController.calculate(Odometry.getInstance().getOdometry(), goal, target);

            driveFromChassis(desiredSpeeds);

            SmartDashboard.putNumber("total time", currTrajectory.getTotalTimeSeconds());

            if(autoController.atReference()&&getAutoTime()>= currTrajectory.getTotalTimeSeconds()){
                setDriveState(DriveState.DONE);
            }

        }

        finally{
            currentAutoTrajectoryLock.unlock();
        }
    }

    public synchronized void setAutoPath(Trajectory trajectory){
        currentAutoTrajectoryLock.lock();
        try{
            rotController.reset(Odometry.getInstance().getOdometry().getRotation().getRadians());
            setDriveState(DriveState.AUTO);
            this.currTrajectory = trajectory;
            autoStartTime = Timer.getFPGATimestamp();
        }

        finally{
            currentAutoTrajectoryLock.unlock();
        }
    }

    public synchronized void setAutoRotation(Rotation2d heading){
        desiredAutoHeading = heading;
    }

    public synchronized double getAutoTime(){
        return Timer.getFPGATimestamp()-autoStartTime;
    }

    public synchronized void stopMotors(){
        driveFromChassis(new ChassisSpeeds(0, 0, 0));
    }

    private void setModuleStates(SwerveModuleState[] states){
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        rearLeft.setState(states[2]);
        rearRight.setState(states[3]);

    }

    public synchronized SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = {frontLeft.getState(),frontRight.getState(),rearLeft.getState(),rearRight.getState()};
        return states;
    }


    public synchronized Rotation2d getDriveHeading(){
        return gyro.getRotation2d();
    }

    public synchronized void calibrateGyro(){
        gyro.calibrate();
    }

    public synchronized void resetHeading(){
        gyro.reset();
    }

    public synchronized boolean isFinished(){
        return getDriveState()==DriveState.DONE || getDriveState() == DriveState.TELE;
    }
    
    public synchronized void setDriveState(DriveState state){

        driveState= state;
    }

    private DriveState getDriveState(){
        return driveState;
    }

    public synchronized void setTeleop(){
        driveState = DriveState.TELE;
    }

    public synchronized void setVisionAlign(){
        driveState = DriveState.VISION;
    }

    public synchronized void togggleRotationSpeed(boolean max){
        if(max)
            maxAngVel = 1.5*Math.PI;
        else
            maxAngVel = .85*Math.PI;

    }

    public synchronized boolean isAligned(){
        if(VisionManager.getInstance().hasVisionTarget()){
            Rotation2d onTarget = Rotation2d.fromDegrees(3);
            double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).getRadians();
            return(Math.abs(error)<Units.degreesToRadians(3));
        }
        else
            return false;
           
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void logData() {
        SmartDashboard.putString("Drive State", getDriveState().toString());
        SmartDashboard.putNumber("Gyro Angle", -gyro.getAngle());

        SmartDashboard.putNumber("left front", frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("left rear", rearLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("right rear", rearRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("front right", frontRight.getState().angle.getDegrees());

        SmartDashboard.putNumber("rotation speed", maxAngVel);

        SmartDashboard.putBoolean("at reference",autoController.atReference());
    }

}
