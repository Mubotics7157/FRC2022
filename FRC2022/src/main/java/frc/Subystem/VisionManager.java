package frc.Subystem;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Threading.Threaded;

public class VisionManager extends Threaded{
    static VisionManager instance;
    PhotonCamera camera = new PhotonCamera("limelight");
    VisionState visionState =VisionState.ON;
    double yaw; 
        double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    double tgtXPos = Units.feetToMeters(54);
    double tgtYPos =
            Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    Pose2d farTargetPose = new Pose2d(new Translation2d(tgtXPos, tgtYPos), new Rotation2d(0.0));
    public SimVisionSystem simVision =
            new SimVisionSystem(
                    "photonvision",
                    75.7607,
                    Units.degreesToRadians(-2), 
                    new Transform2d(),
                    1.55,
                    20,
                    320,
                    240,
                    0);


    public VisionManager(){
        simVision.addSimVisionTarget(
            new SimVisionTarget(farTargetPose, targetHeight, targetWidth, targetHeight));
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

    public synchronized double getDistance(){
        var result = camera.getLatestResult();
        return PhotonUtils.calculateDistanceToTargetMeters(.55, 1, Units.degreesToRadians(-2), Units.degreesToRadians(result.getBestTarget().getPitch()));
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
        simVision.processFrame(Drive.getInstance().getSimPose());

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
