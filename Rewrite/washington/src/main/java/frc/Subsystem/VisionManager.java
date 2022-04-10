package frc.Subsystem;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    double lastKnownDistance = 0;
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

    public synchronized void enableLimelight(){
        tableLife.getEntry("ledMode").setNumber(0);
    }

    public synchronized void disableLimelight(){
        tableLife.getEntry("ledMode").setNumber(1);
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
        if(hasVisionTarget()&&TargetPitch!=0){
            double distance = Units.metersToInches(Constants.VisionConstants.TARGET_HEIGHT_METERS - Constants.VisionConstants.CAM_HEIGHT_METERS) / Math.tan(Constants.VisionConstants.CAM_MOUNTING_PITCH_RADIANS + Units.degreesToRadians(TargetPitch));
            if(distance<5)
                lastKnownDistance = distance;

        if(distance > 1.96 && distance < 2.82)
            LED.getInstance().setGREEN();
            //^^ if bot is within bagel led will turn green
            //^^ if not then stay orang

        else
            LED.getInstance().setORANGE();
        
            return Units.inchesToMeters(distance);
        }
        else 
            return lastKnownDistance;
 
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


        /*
        if(getDistanceToTarget() > 1.96 && getDistanceToTarget() < 2.82)
            LED.getInstance().setGREEN();
            //^^ if bot is within bagel led will turn green
            //^^ if not then stay orang

        else
            LED.getInstance().setORANGE();
        
            */
        
    }
    @Override
    public void selfTest() {}

    @Override
    public void logData() {
        SmartDashboard.putNumber("distance to target", getDistanceToTarget());
        SmartDashboard.putBoolean("target found?", hasVisionTarget());
    }

}
