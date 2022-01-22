package frc.Subystem.SwerveDrive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ModuleHelper {
    double strafe;
    double forward;
    double[] translationalVectors = new double[4];
    double length = Constants.DriveConstants.WHEELBASE_LENGTH;
    double width = Constants.DriveConstants.WHEELBASE_WIDTH;
    double diameter = Constants.DriveConstants.WHEELBASE_DIAMETER;
 
    
    public void transformFieldCentric(double FWD, double STR, double theta){
        forward = (FWD*Math.cos(Units.degreesToRadians(theta))) + (STR*Math.sin(Units.degreesToRadians(theta)));
        strafe = (STR*Math.cos(Units.degreesToRadians(theta))) - (FWD*Math.sin(Units.degreesToRadians(theta)));
        SmartDashboard.putNumber("transformed strafe", strafe);
        SmartDashboard.putNumber("transformed forward", forward);
    }

    public double[] calculateWheelSignals(){

        double[] wheelSignals = new double[4];
        wheelSignals[0] = Math.sqrt(Math.pow(translationalVectors[1],2)+Math.pow(translationalVectors[3],2));
        wheelSignals[1] = Math.sqrt(Math.pow(translationalVectors[1],2)+Math.pow(translationalVectors[2], 2));
        wheelSignals[2] = Math.sqrt(Math.pow(translationalVectors[0],2)+Math.pow(translationalVectors[2], 2));
        wheelSignals[3] = Math.sqrt(Math.pow(translationalVectors[0],2)+Math.pow(translationalVectors[3], 2));

        double maxVal = Math.max(Math.max(Math.abs(wheelSignals[0]), Math.abs(wheelSignals[1])),Math.max(Math.abs(wheelSignals[2]), Math.abs(wheelSignals[3])));
        if(maxVal>1){
        wheelSignals[0] /=maxVal;
        wheelSignals[1] /=maxVal;
        wheelSignals[2] /=maxVal;
        wheelSignals[3] /=maxVal;
        }
        SmartDashboard.putNumber("front left", wheelSignals[0]);
        SmartDashboard.putNumber("front right", wheelSignals[1]);
        SmartDashboard.putNumber("back left", wheelSignals[2]);
        SmartDashboard.putNumber("back right", wheelSignals[3]);

        return wheelSignals;
    }

    public double[] calculateAzimuthAngles(){
        double[] azimuthAngles = new double[4];

        azimuthAngles[0] = Math.atan2(translationalVectors[1], translationalVectors[3]);
        azimuthAngles[1] = Math.atan2(translationalVectors[1], translationalVectors[3]);
        azimuthAngles[2] = Math.atan2(translationalVectors[0], translationalVectors[2]);
        azimuthAngles[3] = Math.atan2(translationalVectors[0], translationalVectors[3]);
        return azimuthAngles;
    }

    public void updateTranslationalVectors(double FWD, double STR, double theta, double rotation){
        transformFieldCentric(FWD, STR, theta);
        translationalVectors[0] = strafe - rotation * (length/diameter);
        translationalVectors[1] = strafe + rotation * (length/diameter);
        translationalVectors[2] = forward - rotation * (width/diameter);
        translationalVectors[3] = forward + rotation * (width/diameter);
    }
    
}
