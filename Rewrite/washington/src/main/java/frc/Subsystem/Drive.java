package frc.Subsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.util.AbstractSubsystem;

public class Drive extends AbstractSubsystem{
    
    public enum DriveState{
        FIELD_ORIENTED,
        ROBOT_ORIENTED,
        VISION,
        AUTO,
        DONE
    }

    private static DriveState driveState = DriveState.ROBOT_ORIENTED;
    private static final Drive driveInstance = new Drive();

    private Module frontLeft = new Module(1,2,3,0);
    private Module frontRight = DriveConstants.FRONT_RIGHT_MODULE;
    private Module rearRight = DriveConstants.REAR_RIGHT_MODULE;
    private Module rearLeft = DriveConstants.REAR_LEFT_MODULE;

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    private Translation2d centerOfRotation = new Translation2d();


    TrapezoidProfile.Constraints visionRotProfile = new TrapezoidProfile.Constraints(4,4);
    ProfiledPIDController visionRotController = new ProfiledPIDController(DriveConstants.TURN_kP, 0, DriveConstants.TURN_kD,visionRotProfile);
        
    private Drive() {
        super(20,20);
        visionRotController.enableContinuousInput(-Math.PI, Math.PI);
        visionRotController.setTolerance(Units.degreesToRadians(3));
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
            case FIELD_ORIENTED:
                updateManual(true);
                break;
            case ROBOT_ORIENTED:
                updateManual(false);
                break;
            case VISION:
                break;
            case AUTO:
                break;
            case DONE:
                break;
        }
    }

    private void updateManual(boolean fieldOriented){
        double str = Robot.driver.getLeftX();
        double fwd = Robot.driver.getLeftY();
        double rot = Robot.driver.getRightX();

        if(Math.abs(str) < .1)
            str = 0;
        if(Math.abs(fwd) < .1)
            fwd = 0;
        if(Math.abs(rot) < .15)
            rot = 0;
        if(fieldOriented)
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(fwd*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
            str*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
            rot*DriveConstants.MAX_TELE_ANGULAR_VELOCITY,
            getDriveHeading()));
        else  
            driveFromChassis(new ChassisSpeeds (fwd*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
            str*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY,
            rot*DriveConstants.MAX_TELE_ANGULAR_VELOCITY));
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
            rot*DriveConstants.MAX_TELE_ANGULAR_VELOCITY,
            getDriveHeading()));
    }

    private void updateAlign(){
        if(VisionManager.getInstance().hasVisionTarget()){
        Rotation2d onTarget = new Rotation2d(0);
        double error = onTarget.rotateBy(VisionManager.getInstance().getTargetYawRotation2d()).getRadians();
        if(Math.abs(error)<Units.degreesToRadians(3))
            error = 0;
        double deltaSpeed = visionRotController.calculate(error);
        updateManual(true,deltaSpeed);
        //}
       //else{
           if(visionRotController.atGoal())
            setDriveState(DriveState.FIELD_ORIENTED);
        }
    }

    private void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(getModuleStates(), DriveConstants.MAX_TANGENTIAL_VELOCITY);
        setModuleStates(states);


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

    public synchronized void setCenterOfRotation(Translation2d center){
        centerOfRotation = center;
    }

    public synchronized Rotation2d getDriveHeading(){
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public synchronized void calibrateGyro(){
        gyro.calibrate();
    }

    public synchronized void resetHeading(){
        gyro.reset();
        Odometry.getInstance().resetHeading();
    }

    public synchronized double getDriveRoll(){
        return gyro.getRoll();
    }
    
    public synchronized void setDriveState(DriveState state){
        driveState= state;
    }

    private DriveState getDriveState(){
        return driveState;
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

    }


}
