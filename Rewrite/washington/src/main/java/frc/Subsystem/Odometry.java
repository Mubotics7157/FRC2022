package frc.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.util.AbstractSubsystem;

public class Odometry extends AbstractSubsystem{

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading());
    
    private static Odometry instance = new Odometry();

    private Odometry(){
        super(20,20);
    }

    public static Odometry getInstance(){
        return instance;
    }

    @Override
    public void update() {
        odometry.update(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModuleStates());
    }

    public synchronized void setOdometry(Pose2d pose){
        odometry.resetPosition(pose, Drive.getInstance().getDriveHeading());  
    }

    public synchronized Pose2d getOdometry(){
        return odometry.getPoseMeters();
    }

    public synchronized void resetHeading(){
        odometry.resetPosition(new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("Pose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Pose R", odometry.getPoseMeters().getRotation().getDegrees());
    }

    @Override
    public void selfTest() {
        
    }
}
