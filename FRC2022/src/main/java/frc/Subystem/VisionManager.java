package frc.Subystem;


import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.util.Threading.Threaded;

public class VisionManager extends Threaded{
    static VisionManager instance;
    public PhotonCamera targetCam;
    PhotonCamera cargoCam;
    VisionState visionState =VisionState.ON;
    double yaw; 
 
    public VisionManager(){
        targetCam = new PhotonCamera("gloworm");
        cargoCam = new PhotonCamera("LifeCam");
    }

    public enum VisionState{
        OFF,
        ON
    }

    public synchronized double getTargetYaw(){
        var result = targetCam.getLatestResult();
        if(result.hasTargets()){
            return -result.getBestTarget().getYaw();
        }
        else
            return 0;
    }

    public synchronized Rotation2d getTargetYawRotation2d(){
        var result = targetCam.getLatestResult();
        if(result.hasTargets()){
            Rotation2d yaw = new Rotation2d(Units.degreesToRadians(result.getBestTarget().getYaw()));
            return yaw.unaryMinus();
        }
        else
            return new Rotation2d(0);
    }
    
    public synchronized Rotation2d getCargoYawRotation2d(){
        var result = cargoCam.getLatestResult();
        if(result.hasTargets()){
            Rotation2d yaw = new Rotation2d(Units.degreesToRadians(result.getBestTarget().getYaw()));
            return yaw.unaryMinus();
        }
        else
            return new Rotation2d(0);
    }

    public synchronized boolean hasVisionTarget(){
        var result = targetCam.getLatestResult();
        return result.hasTargets();
    }

    public synchronized double getDistanceToTarget(){
        var result = targetCam.getLatestResult();
        double TargetPitch = result.getBestTarget().getPitch();

        double distance = Units.metersToInches(Constants.VisionConstants.CAMERA_HEIGHT_METERS - Constants.VisionConstants.TARGET_HEIGHT_METERS)/Math.tan(Units.degreesToRadians(Constants.VisionConstants.CAMERA_PITCH_RADIANS + TargetPitch));
        return distance;
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
        
    }
    
    public synchronized void setOff(){
        targetCam.setLED(VisionLEDMode.kOff);
        targetCam.setDriverMode(true);
        cargoCam.setDriverMode(true);
    }

    public synchronized void setOn(){
        targetCam.setDriverMode(false);
        targetCam.setLED(VisionLEDMode.kOn);
        cargoCam.setDriverMode(false);
    }

    public synchronized void toggleOnOff(){
        if(visionState==VisionState.ON)
            setOn();
        else
            setOff();
    }
}