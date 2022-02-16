package frc.Subystem;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.robot.Constants.FieldConstants;
import frc.util.Threading.Threaded;

public class VisionManager extends Threaded{
    static VisionManager instance;
    PhotonCamera camera;
    VisionState visionState =VisionState.ON;
    double yaw; 
    SimVisionSystem simVisionSystem;
    SimVisionTarget powerPort;
    double camDiagFOV = 170.0; 
    double camPitch = 1;
    double camHeightOffGround = Units.inchesToMeters(24); // meters
    double maxLEDRange = 20;
    int camResolutionWidth = 640;
    int camResolutionHeight = 480;
    double minTargetArea = 10;
 
    public VisionManager(){
        camera = new PhotonCamera("gloworm");

        SimVisionSystem simVision =
            new SimVisionSystem(
                    "photonvision",
                    camDiagFOV,
                    camPitch,
                    new Transform2d(),
                    camHeightOffGround,
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);
                
        powerPort = new SimVisionTarget(FieldConstants.farTargetPose, FieldConstants.targetHeight, FieldConstants.targetWidth, FieldConstants.targetWidth);

        simVision.addSimVisionTarget(powerPort);
    }
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

    public synchronized Rotation2d getYawRotation2d(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            Rotation2d yaw = new Rotation2d(Units.degreesToRadians(result.getBestTarget().getYaw()));
            return yaw.unaryMinus();
        }
        else
            return new Rotation2d(0);
    }

    public synchronized double getRange(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            double range = PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);
            return range;
        }
        
        else return 0;

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
        simVisionSystem.processFrame(SwerveTracker.getInstance().getOdometry());
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
