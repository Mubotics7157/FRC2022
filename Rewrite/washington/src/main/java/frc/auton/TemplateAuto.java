package frc.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.Subsystem.Odometry;


public abstract class TemplateAuto implements Runnable {
    protected boolean done = false;


    Odometry odometry = Odometry.getInstance();


    public TemplateAuto() {
    }

    public Translation2d here() {
        return Odometry.getInstance().getOdometry().getTranslation();
    }

    public Rotation2d dir() {
        return Odometry.getInstance().getOdometry().getRotation();
    }


    synchronized public boolean isFinished() {
        return done;
    }

    public synchronized void reset() {
        this.done = false;
    }
}
