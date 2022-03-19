package frc.Subsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.util.AbstractSubsystem;

public class VisionManager extends AbstractSubsystem{

    static VisionManager instance;
    PhotonCamera targetCam;
    PhotonCamera cargoCam;
    VisionState visionState =VisionState.ON;
    double yaw; 
 
    public VisionManager(){
        super(20);
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
        if(result.hasTargets()){
        double TargetPitch = -result.getBestTarget().getPitch();

        double distance = Units.metersToInches(Constants.VisionConstants.CAM_HEIGHT_METERS - Constants.VisionConstants.TARGET_HEIGHT_METERS)/Math.tan(Units.degreesToRadians(Constants.VisionConstants.CAM_MOUNTING_PITCH_RADIANS + TargetPitch));
        return Units.inchesToMeters(distance);
         }

         else
            return 0;
    }

    public synchronized double getPhotonDistanceToTarget(){
        var result = targetCam.getLatestResult();
        if(result.hasTargets())
        return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAM_HEIGHT_METERS, VisionConstants.TARGET_HEIGHT_METERS, VisionConstants.CAM_MOUNTING_PITCH_RADIANS,Units.degreesToRadians(result.getBestTarget().getPitch()))   ;
        else
            return 0;
    }

    public synchronized double getCargoDistance(){
        var result = targetCam.getLatestResult();
        return result.getBestTarget().getPitch();
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
            case ON:
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

    @Override
    public void selfTest() {}

    @Override
    public void logData() {
        SmartDashboard.putNumber("distance to target", getDistanceToTarget());
        SmartDashboard.putNumber("photon distance to target", getPhotonDistanceToTarget());
    }

}
