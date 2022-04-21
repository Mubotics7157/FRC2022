package frc.Subsystem;

import java.lang.reflect.Field;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subsystem.Drive.DriveState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.util.AbstractSubsystem;

public class Odometry extends AbstractSubsystem{

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading());
    Field2d m_field;
    private static Odometry instance = new Odometry();

    Matrix<N3,N1> stateDev;
    Matrix<N1,N1> localDev;
    Matrix<N3,N1> visionDev;

    private Odometry(){
        super(20,20);
        stateDev = VecBuilder.fill(.02, .02, .05);
        localDev = VecBuilder.fill(.02);
        visionDev = VecBuilder.fill(.05,.05,.05);
        
        SmartDashboard.putData("field", m_field);

    }

    public static Odometry getInstance(){
        return instance;
    }

    @Override
    public void update() {
        odometry.update(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModuleStates());
        //odometry.updateWithTime(Timer.getFPGATimestamp(), Drive.getInstance().getDriveHeading(), Drive.getInstance().getModuleStates());
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
