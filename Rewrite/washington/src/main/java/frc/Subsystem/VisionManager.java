package frc.Subsystem;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.util.AbstractSubsystem;

public class VisionManager extends AbstractSubsystem{

    static VisionManager instance;
    NetworkTable tableLime;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tv;
    NetworkTable tableLife;
    VisionState visionState =VisionState.ON;
    double yaw; 
    boolean TargetFound;
 
    public VisionManager(){
        super(20);
        tableLime = NetworkTableInstance.getDefault().getTable("limelight");
        tableLife = NetworkTableInstance.getDefault().getTable("lifecam");
    }

    public enum VisionState{
        OFF,
        ON
    }

    public synchronized double getTargetYaw(){
        if(tableLime.getEntry("tv").getDouble(0) == 1){
            return -tableLime.getEntry("tx").getDouble(0);
        }
        else
            return 0;
    }

    public synchronized Rotation2d getTargetYawRotation2d(){
        if(tableLime.getEntry("tv").getDouble(0) == 1){
            Rotation2d yaw = new Rotation2d(Units.degreesToRadians(tableLime.getEntry("tx").getDouble(0)));
            return yaw.unaryMinus();
        }
        else
            return new Rotation2d(0);
    }
    
    public synchronized Rotation2d getCargoYawRotation2d(){
        if(tableLife.getEntry("tv").getDouble(0) == 1){
            Rotation2d yaw = new Rotation2d(Units.degreesToRadians(tableLife.getEntry("tx").getDouble(0)));
            return yaw.unaryMinus();
        }
        else
            return new Rotation2d(0);
    }

    public synchronized boolean hasVisionTarget(){
        if(tableLime.getEntry("tv").getDouble(0) == 1)
        TargetFound = true;
        else
        TargetFound = false;
        return TargetFound;
    }

    public synchronized double getDistanceToTarget(){
        double TargetPitch = tableLime.getEntry("ty").getDouble(0);

        double distance = Units.metersToInches(Constants.VisionConstants.CAM_HEIGHT_METERS - Constants.VisionConstants.TARGET_HEIGHT_METERS)/Math.tan(Units.degreesToRadians(Constants.VisionConstants.CAM_MOUNTING_PITCH_RADIANS + TargetPitch));
        return distance;
    }

    public synchronized double getCargoDistance(){
        return tableLime.getEntry("ty").getDouble(0);
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
   /* 
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
*/
    @Override
    public void selfTest() {}

    @Override
    public void logData() {}

}
