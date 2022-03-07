package frc.Subystem.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.util.Threading.Threaded;

public class SwerveTracker extends Threaded{

    

    SwerveDrive swerve  = SwerveDrive.getInstance();

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.SWERVE_KINEMATICS, swerve.getDriveHeading());

    private static final SwerveTracker instance = new SwerveTracker();

    public static SwerveTracker getInstance(){
        return instance;
    }

    @Override
    public void update() {
        odometry.update(swerve.getDriveHeading(), swerve.getModuleStates());
        SmartDashboard.putNumber("pose x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("pose y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("pose r", odometry.getPoseMeters().getRotation().getDegrees());
    }

    public synchronized void setOdometry(Pose2d pose){
        odometry.resetPosition(pose, pose.getRotation());  
    }

    public synchronized Pose2d getOdometry(){
        return odometry.getPoseMeters();
    }
}
