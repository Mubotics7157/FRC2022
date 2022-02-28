package frc.Subystem.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.util.Threading.Threaded;

public class SwerveTracker extends Threaded{

    Field2d field = new Field2d();
    

    SwerveDrive swerve  = SwerveDrive.getInstance();

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.SWERVE_KINEMATICS, swerve.getDriveHeading());

    Pose2d[] modulePoses = {
        new Pose2d(),
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };

    private static SwerveTracker instance;

    public static SwerveTracker getInstance(){
        if(instance==null)
            instance = new SwerveTracker();
        return instance;
    }

    @Override
    public void update() {
        updateOdometry();
        synchronized(this){
            SmartDashboard.putNumber("PoseX", getOdometry().getX());
            SmartDashboard.putNumber("PoseY", getOdometry().getY());
            SmartDashboard.putNumber("PoseR", getOdometry().getRotation().getDegrees());
        }
    }

    private void updateOdometry(){
        odometry.update(swerve.getDriveHeading(), swerve.getModuleStates());
    }

    public synchronized void setOdometry(Pose2d pose){
        odometry.resetPosition(pose, pose.getRotation());  
    }

    public synchronized void resetOdometry(Pose2d pose){
        odometry.resetPosition(new Pose2d(0,0,Rotation2d.fromDegrees(0)),Rotation2d.fromDegrees(0));
    }
    public synchronized Pose2d getOdometry(){
        return odometry.getPoseMeters();
    }
}
