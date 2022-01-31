package frc.Subystem.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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
    }

    private void updateOdometry(){
        odometry.update(swerve.getDriveHeading(), swerve.getModuleStates());
        SmartDashboard.putData(field);
        field.setRobotPose(odometry.getPoseMeters());
        for (int i = 0; i < DriveConstants.MODULE_POSITIONS.length; i++) {
            Translation2d modulePositionFromChassis = DriveConstants.MODULE_POSITIONS[i]
            .rotateBy(swerve.getDriveHeading())
            .plus(odometry.getPoseMeters().getTranslation());

            modulePoses[i] = new Pose2d(modulePositionFromChassis, swerve.getModuleStates()[i].angle.plus(odometry.getPoseMeters().getRotation()));
        }
       field.getObject("Modules").setPoses(modulePoses);
    }

    public synchronized void setOdometry(Pose2d pose){
        odometry.resetPosition(pose, pose.getRotation());
    }

    public synchronized Pose2d getOdometry(){
        return odometry.getPoseMeters();
    }
}
