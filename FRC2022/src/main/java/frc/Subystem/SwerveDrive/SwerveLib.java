package frc.Subystem.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveLib {
    static double speedFR;
    static double speedFL;
    static double speedBR;
    static double speedBL;
    public SwerveModuleState[] calcFieldCentricInputs(double fwd, double str, double rot, Rotation2d heading){
        SwerveModuleState[] states = new SwerveModuleState[4];
        double nontransformed = fwd;
        fwd = fwd * Math.cos(heading.getRadians()) + str*Math.sin(heading.getRadians());
        str = str * Math.cos(heading.getRadians()) - nontransformed*Math.sin(heading.getRadians());

        double a = str-rot * (DriveConstants.WHEELBASE_LENGTH/DriveConstants.WHEELBASE_RADIUS);
        double b = str+rot * (DriveConstants.WHEELBASE_LENGTH/DriveConstants.WHEELBASE_RADIUS);
        double c = fwd-rot * (DriveConstants.WHEELBASE_WIDTH/DriveConstants.WHEELBASE_RADIUS);
        double d = str+rot * (DriveConstants.WHEELBASE_LENGTH/DriveConstants.WHEELBASE_RADIUS);

        speedFR = Math.sqrt(Math.pow(b, 2)+ Math.pow(c, 2));
        speedFL = Math.sqrt(Math.pow(b, 2)+ Math.pow(d, 2));
        speedBR = Math.sqrt(Math.pow(a, 2)+ Math.pow(c, 2));
        speedBL = Math.sqrt(Math.pow(a, 2)+ Math.pow(d, 2));

        normalizeSpeeds();

        states[0] =new SwerveModuleState(speedFL, new Rotation2d(Math.atan2(b, d)));
        states[1] =new SwerveModuleState(speedFR, new Rotation2d(Math.atan2(b, c)));
        states[2] =new SwerveModuleState(speedBL, new Rotation2d(Math.atan2(b, d)));
        states[3] =new SwerveModuleState(speedBR, new Rotation2d( Math.atan2(a, c)));
        return states;

        }

    public void normalizeSpeeds(){
        double maxSpeed = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));
        if(maxSpeed>1){
            speedBL/=maxSpeed;
            speedBR/=maxSpeed;
            speedFR/=maxSpeed;
            speedFL/=maxSpeed;
        }
        speedBL*= DriveConstants.MAX_TANGENTIAL_VELOCITY;
        speedBR*= DriveConstants.MAX_TANGENTIAL_VELOCITY;
        speedFR*= DriveConstants.MAX_TANGENTIAL_VELOCITY;
        speedFL*= DriveConstants.MAX_TANGENTIAL_VELOCITY;
    }

}
