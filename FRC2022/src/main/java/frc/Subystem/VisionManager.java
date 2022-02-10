package frc.Subystem;


import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.util.Threading.Threaded;

public class VisionManager extends Threaded{
    static VisionManager instance;
    PhotonCamera camera = new PhotonCamera("limelight");
    VisionState visionState =VisionState.ON;
    double yaw; 

    public enum VisionState{
        OFF,
        ON
    }


    public synchronized double getTargetYaw(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            return -result.getBestTarget().getYaw();
        }
        else
            return 0;
    }

    public synchronized double getDistance(){
        var result = camera.getLatestResult();
        double TargetPitch = result.getBestTarget().getPitch();

        double distance = Units.metersToInches(Constants.VisionConstants.CameraHeight - Constants.VisionConstants.TargetHeight)/Math.tan(Units.degreesToRadians(Constants.VisionConstants.CameraPitch + TargetPitch));
        return distance;
    }

    public synchronized Rotation2d getYawRotation2d(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            Rotation2d yaw = new Rotation2d(Units.degreesToRadians(result.getBestTarget().getYaw()));
            return yaw.unaryMinus();
        }
        else
            return new Rotation2d(0);
    }

    public synchronized Translation2d getVisionTrans(){
        return PhotonUtils.estimateCameraToTargetTranslation(getDistance(), getYawRotation2d());
    }

    public static VisionManager getInstance(){
        if(instance == null)
            instance = new VisionManager();
            
        return instance;
    }

    @Override
    public void update() {
        VisionState snapVisionState;
        synchronized(this){
            snapVisionState = visionState;
        }

        switch(snapVisionState){
            case OFF:
                SmartDashboard.putString("vision state", "off");
            case ON:
                SmartDashboard.putString("vision state", "on");
        }
        SmartDashboard.putNumber("yaw", getTargetYaw());
    }
    
    public synchronized void setOff(){
        camera.setLED(VisionLEDMode.kOff);
        camera.setDriverMode(true);
    }

    public synchronized void setOn(){
        camera.setDriverMode(false);
        camera.setLED(VisionLEDMode.kOn);
    }
}
