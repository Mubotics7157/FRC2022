// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subsystem.Drive;
import frc.Subsystem.Odometry;
import frc.Subsystem.Drive.DriveState;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.OsUtil;
import frc.auton.guiauto.serialization.reflection.ClassInformationSender;
import frc.util.OrangeUtility;

public class Robot extends TimedRobot {

    //GUI
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable autoDataTable = instance.getTable("autodata");
    NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");

    NetworkTable position = autoDataTable.getSubTable("position");
    NetworkTableEntry xPos = position.getEntry("x");
    NetworkTableEntry yPos = position.getEntry("y");
    NetworkTableEntry enabled = autoDataTable.getEntry("enabled");
    NetworkTableEntry pathProcessingStatusEntry = autoDataTable.getEntry("processing");
    NetworkTableEntry pathProcessingStatusIdEntry = autoDataTable.getEntry("processingid");

    //auto
    TemplateAuto selectedAuto;
    Thread autoThread;
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "My Auto";
    private final SendableChooser<String> autoChooser = new SendableChooser<>();


    private final Lock networkAutoLock = new ReentrantLock();
    NetworkAuto networkAuto;

    String lastAutoPath = null;

    ExecutorService deserializerExecutor = Executors.newSingleThreadExecutor();

    Consumer<EntryNotification> autoPathListener = (event ->
            deserializerExecutor.execute(() -> { //Start deserializing on another thread
                        System.out.println("starting to parse autonomous");
                        //Set networktable entries for the gui notifications
                        pathProcessingStatusEntry.setDouble(1);
                        pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
                        networkAutoLock.lock();
                        try {
                            networkAuto = new NetworkAuto(); //Create the auto object which will start deserializing the json
                            // and the auto
                        } finally {
                            networkAutoLock.unlock();
                        }

                        // ready to be run
                        System.out.println("done parsing autonomous");
                        //Set networktable entries for the gui notifications
                        pathProcessingStatusEntry.setDouble(2);
                        pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
                    }
            ));

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static XboxController driver = new XboxController(0);
  public static Joystick operator = new Joystick(1);
  public Drive swerve = Drive.getInstance();
  public Odometry odometry = Odometry.getInstance();

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    swerve.start();
    swerve.calibrateGyro();
    OrangeUtility.sleep(50);
    swerve.resetHeading();
    odometry.start();
  }

  @Override
  public void robotPeriodic() {
      if (isEnabled()) {
        //Get data from the robot tracker and upload it to the robot tracker (Units must be in meters)
        xPos.setDouble(odometry.getOdometry().getX());
        yPos.setDouble(odometry.getOdometry().getY());
      }

        //Listen changes in the network auto
      if (autoPath.getString(null) != null && !autoPath.getString(null).equals(lastAutoPath)) {
        lastAutoPath = autoPath.getString(null);
        deserializerExecutor.execute(() -> { //Start deserializing on another thread
         System.out.println("start parsing autonomous");
        //Set networktable entries for the gui notifications
        pathProcessingStatusEntry.setDouble(1);
        pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);

        networkAuto = new NetworkAuto(); //Create the auto object which will start deserializing the json and the auto
        // ready to be run
        System.out.println("done parsing autonomous");
        //Set networktable entries for the gui notifications
        pathProcessingStatusEntry.setDouble(2);
        pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
        });
      }
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    networkAutoLock.lock();
       try {
            if (networkAuto == null) {
                System.out.println("Using normal autos");
                String auto = autoChooser.getSelected();
                switch (auto) {
                    //Put all your autos here
                }
            } else {
                System.out.println("Using autos from network tables");
                selectedAuto = networkAuto;
            }
        } finally {
            networkAutoLock.unlock();
        }
        
        assert selectedAuto != null;
        //Since autonomous objects can be reused they need to be reset them before we can reuse them again 
        selectedAuto.reset();

        //We then create a new thread to run the auto and run it
        autoThread = new Thread(selectedAuto);
        autoThread.start();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    if(autoPath.getString(null)!=null)
      autoPathListener.accept(new EntryNotification(NetworkTableInstance.getDefault(),1,1,"",null,12));
    
    autoPath.addListener(autoPathListener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    killAuto();
    enabled.setBoolean(true);
    swerve.resetHeading();
    swerve.setDriveState(DriveState.FIELD_ORIENTED);

  }

  @Override
  public void teleopPeriodic() {
    if(driver.getLeftBumper())
      swerve.resetHeading();
  }

  @Override
  public void disabledInit() {
    killAuto();
    enabled.setBoolean(false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}


  @Override
  public void simulationInit() {
      ClassInformationSender.updateReflectionInformation(
              new File(OsUtil.getUserConfigDirectory("AutoBuilder") + "/robotCodeData.json"));
  }

  @Override
  public void simulationPeriodic() {}

  public synchronized void killAuto() {
    System.out.println("Killing Auto");
    if (selectedAuto != null) {
        assert autoThread != null;
        autoThread.interrupt();
        double nextStackTracePrint = Timer.getFPGATimestamp() + 1;
        while (!(selectedAuto.isFinished() || autoThread.getState() == Thread.State.TERMINATED)) {
            if (Timer.getFPGATimestamp() > nextStackTracePrint) {
                Exception throwable = new Exception(
                        "Waiting for auto to die. selectedAuto.isFinished() = " + selectedAuto.isFinished() +
                                " autoThread.getState() = " + autoThread.getState());
                throwable.setStackTrace(autoThread.getStackTrace());
                throwable.printStackTrace();
                nextStackTracePrint = Timer.getFPGATimestamp() + 5;
            }


            OrangeUtility.sleep(10);
        }
        swerve.stopMotors();
        swerve.setDriveState(DriveState.FIELD_ORIENTED);
    }
  }
}