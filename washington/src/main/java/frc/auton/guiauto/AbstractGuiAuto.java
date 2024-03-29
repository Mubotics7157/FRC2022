package frc.auton.guiauto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.Subsystem.Drive;
import frc.Subsystem.Odometry;
import frc.Subsystem.Drive.DriveState;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.AbstractAutonomousStep;
import frc.auton.guiauto.serialization.Autonomous;
import frc.auton.guiauto.serialization.TrajectoryAutonomousStep;
import frc.auton.guiauto.serialization.command.CommandExecutionFailedException;
import frc.auton.guiauto.serialization.command.SendableScript;
import frc.util.Serializer;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

//If your autos don't have a superclass that they extend you can replace TemplateAuto with Runnable
public abstract class AbstractGuiAuto extends TemplateAuto {

    private Autonomous autonomous;
    Pose2d initialPose;

    /**
     * Ensure you are creating the objects for your auto on robot init. The roborio will take multiple seconds to initialize the
     * auto.
     *
     * @param autonomousFile File location of the auto
     */
    public AbstractGuiAuto(File autonomousFile) {
        try {
            autonomous = (Autonomous) Serializer.deserializeFromFile(autonomousFile, Autonomous.class);
        } catch (IOException e) {
            //e.printStackTrace();
            DriverStation.reportError("Failed to deserialize auto. " + e.getLocalizedMessage(), e.getStackTrace());
        }
        init();
    }

    /**
     * Ensure you are creating the objects for your auto before you run them. The roborio will take multiple seconds to initialize
     * the auto.
     *
     * @param autonomousJson String of the autonomous
     */
    public AbstractGuiAuto(String autonomousJson) {
        try {
            autonomous = (Autonomous) Serializer.deserialize(autonomousJson, Autonomous.class);
        } catch (IOException e) {
            DriverStation.reportError("Failed to deserialize auto. " + e.getMessage(), e.getStackTrace());
        }
        init();
    }

    private void init() {
        //Find and save the initial pose
        for (AbstractAutonomousStep autonomousStep : autonomous.getAutonomousSteps()) {
            if (autonomousStep instanceof TrajectoryAutonomousStep) {
                TrajectoryAutonomousStep trajectoryAutonomousStep = (TrajectoryAutonomousStep) autonomousStep;
                Trajectory.State initialState = trajectoryAutonomousStep.getTrajectory().getStates().get(0);
                initialPose = new Pose2d(initialState.poseMeters.getTranslation(),
                        trajectoryAutonomousStep.getRotations().get(0).rotation);
                break;
            }
        }
    }

    @Override
    public void run() {
        Thread.currentThread().setUncaughtExceptionHandler((t, e) -> {
            DriverStation.reportError("Uncaught exception in auto thread: " + e.getMessage(), e.getStackTrace());
            Drive.getInstance().stopMotors();
            synchronized (this) {
                this.done = true;
            }
        });

        System.out.println("Started Running: " + Timer.getFPGATimestamp());
        //Set our initial pose in our robot tracker
        if (initialPose != null) {
            Odometry.getInstance().setOdometry(initialPose);
        }

        //Loop though all the steps and execute them
        List<SendableScript> scriptsToExecuteByTime = new ArrayList<>();
        List<SendableScript> scriptsToExecuteByPercent = new ArrayList<>();

        for (AbstractAutonomousStep autonomousStep : autonomous.getAutonomousSteps()) {
            System.out.println("doing a step: " + Timer.getFPGATimestamp());
            if (Thread.interrupted()) {
                System.out.println("Auto was interrupted " + Timer.getFPGATimestamp());
                return;
            }

            try {
                autonomousStep.execute(this, scriptsToExecuteByTime, scriptsToExecuteByPercent);
            } catch (InterruptedException e) {
                System.out.println("Auto was interrupted " + Timer.getFPGATimestamp());
                e.printStackTrace();
                return;
            } catch (CommandExecutionFailedException e) {
                return;
            }
        }

        System.out.println("finished: " + Timer.getFPGATimestamp());
        Drive.getInstance().stopMotors();

        synchronized (this) {
            done = true;
        }
    }
}