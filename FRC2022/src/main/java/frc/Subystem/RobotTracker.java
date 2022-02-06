package frc.Subystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.Threading.Threaded;

public class RobotTracker extends Threaded {

    private static RobotTracker instance;
    private Drive drive = Drive.getInstance();
    private DifferentialDriveOdometry differentialDriveOdometry = new DifferentialDriveOdometry(drive.getRotation2d()); // can I just use the getRotation2d or do I have to create a new rotation 2d object?

    private Pose2d poseMeters;

    public void update(){
        Rotation2d heading = drive.getRotation2d();
        double leftDist = drive.getLeftDistance();
        double rightDist = drive.getRightDistance();

        synchronized(this){
            differentialDriveOdometry.update(heading, leftDist, rightDist);
            poseMeters = differentialDriveOdometry.getPoseMeters();
            
			SmartDashboard.putNumber("PoseX", differentialDriveOdometry.getPoseMeters().getTranslation().getX());
			SmartDashboard.putNumber("PoseY", differentialDriveOdometry.getPoseMeters().getTranslation().getY());
            SmartDashboard.putNumber("PoseR", differentialDriveOdometry.getPoseMeters().getRotation().getDegrees());
       }
    }

    public static RobotTracker getInstance(){
        if(instance == null)
            instance = new RobotTracker();
        return instance;
    }

    public synchronized void setOdometry(Pose2d pose){
        drive.zeroSensors();
        differentialDriveOdometry.resetPosition(pose, pose.getRotation());
        poseMeters = differentialDriveOdometry.getPoseMeters();
    }

    public synchronized Pose2d getOdometry(){
        return poseMeters;
    }

    public synchronized void resetOdometry(){
        drive.zeroSensors();
        differentialDriveOdometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
        poseMeters = differentialDriveOdometry.getPoseMeters();
    }
}
