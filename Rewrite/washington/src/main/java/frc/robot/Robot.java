package frc.robot;

import java.io.File;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.Subsystem.Climb;
import frc.Subsystem.Drive;
import frc.Subsystem.Intake;
import frc.Subsystem.LED;
import frc.Subsystem.Odometry;
import frc.Subsystem.Shooter;
import frc.Subsystem.VisionManager;
//import frc.Subsystem.Climb.ClimbState;
import frc.Subsystem.Drive.DriveState;
import frc.Subsystem.Intake.IntakeState;
import frc.auton.FiveBall;
import frc.auton.TemplateAuto;
import frc.auton.TwoBall;
import frc.auton.WeakSide;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.reflection.ClassInformationSender;
import frc.util.OrangeUtility;
/*
import frc.util.ClimbRoutine.ActuateHigh;
import frc.util.ClimbRoutine.ActuateMid;
import frc.util.ClimbRoutine.ClimbCommand;
import frc.util.ClimbRoutine.ClimbRoutine;
import frc.util.ClimbRoutine.Delay;
*/

public class Robot extends TimedRobot {

    public static XboxController driver = new XboxController(0);
    public static Joystick operator = new Joystick(1);

    //GUI
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable autoDataTable = instance.getTable("autodata");
    NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");

    NetworkTableEntry enabled = autoDataTable.getEntry("enabled");
    NetworkTableEntry pathProcessingStatusEntry = autoDataTable.getEntry("processing");
    NetworkTableEntry pathProcessingStatusIdEntry = autoDataTable.getEntry("processingid");

    private final Lock networkAutoLock = new ReentrantLock();
    NetworkAuto networkAuto;

    String lastAutoPath = null;

    ExecutorService deserializerExecutor = Executors.newSingleThreadExecutor();

    //Auto
    WeakSide weakSideAuto = new WeakSide();
    TwoBall twoBallAuto;
    FiveBall fiveBallAuto; 
    TemplateAuto selectedAuto;
    Thread autoThread;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    //Subsystems
    private final Odometry odometry = Odometry.getInstance();
    private final Drive drive = Drive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final VisionManager vision = VisionManager.getInstance();
    //Climb climb = Climb.getInstance();

    Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);


    //Control loop states
    boolean limelightTakeSnapshots;

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

    //ClimbRoutine routine = new ClimbRoutine();

    //Thread climbRoutine;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        OrangeUtility.sleep(1500);
        try{
            twoBallAuto = new TwoBall();
            fiveBallAuto = new FiveBall();
        }
        finally{}

        SmartDashboard.putNumber("top wheel setpoint", 1250);
        SmartDashboard.putNumber("shooter ratio", 1.08);
        SmartDashboard.putNumber("shot adjustment", .97);
        SmartDashboard.putNumber("bot wheel setpoint", 1250);
        selectedAuto = twoBallAuto;
        if (autoPath.getString(null) != null) {
            autoPathListener.accept(new EntryNotification(NetworkTableInstance.getDefault(), 1, 1, "", null, 12));
       
        }

        autoPath.addListener(autoPathListener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);



        startSubsystems();
        drive.resetHeading();
        OrangeUtility.sleep(50);
        odometry.setOdometry(new Pose2d());
        //routine.addCommands(new ActuateMid(),new Delay(.4),new ActuateHigh(),new ClimbCommand(-642, 427685));
        Intake.getInstance().toggleInterpolated();
        VisionManager.getInstance().toggleLimelight(false);
        autoChooser.setDefaultOption("default","two");
        autoChooser.addOption("five ball","five");
        autoChooser.addOption("weak side","weak");
        SmartDashboard.putData(autoChooser);
        LED.getInstance().setOFF();
    }
    
    @Override
    public void robotPeriodic() {
        
        if (isEnabled()) {
            //Get data from the robot tracker and upload it to the robot tracker (Units must be in meters)
            SmartDashboard.putNumber("X meters", odometry.getOdometry().getX());
            SmartDashboard.putNumber("Y meters", odometry.getOdometry().getY());
        }
        //SmartDashboard.putBoolean("climb routine is null", climbRoutine==null);

        //Listen changes in the network auto
        if (autoPath.getString(null) != null && !autoPath.getString(null).equals(lastAutoPath)) {
            lastAutoPath = autoPath.getString(null);
            deserializerExecutor.execute(() -> { //Start deserializing on another thread
                System.out.println("**************************");
                System.out.println("start parsing autonomous");
                //Set networktable entries for the gui notifications
                pathProcessingStatusEntry.setDouble(1);
                pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
                networkAuto = new NetworkAuto(); //Create the auto object which will start deserializing the json and the auto
                // ready to be run
                System.out.println("**************************");
                System.out.println("done parsing autonomous");
                //Set networktable entries for the gui notifications
                pathProcessingStatusEntry.setDouble(2);
                pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
            });
        }

    }

    @Override
    public void autonomousInit() {
        LED.getInstance().setORANGE();
        VisionManager.getInstance().toggleLimelight(true);
        enabled.setBoolean(true);

    networkAutoLock.lock();
       try {
            if (networkAuto == null) {
                System.out.println("Using normal autos");
            } else {
                System.out.println("Using autos from network tables");
                selectedAuto = networkAuto;
            }
        } finally {
            networkAutoLock.unlock();
        }
        
        if( selectedAuto != null){
            selectedAuto.reset();

            autoThread = new Thread(selectedAuto);
            autoThread.start();
        }

    if(autoPath.getString(null)!=null)
      autoPathListener.accept(new EntryNotification(NetworkTableInstance.getDefault(),1,1,"",null,12));
    
    autoPath.addListener(autoPathListener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        killAuto();
        enabled.setBoolean(true);
        startSubsystems();
        compressor.enableDigital();
        drive.setDriveState(DriveState.FIELD_ORIENTED);
        drive.resetHeading();
        drive.setDriveState(DriveState.FIELD_ORIENTED);
        compressor.enableDigital();
        intake.toggleIntake(true);
        intake.setCargoColor(true);
         //Climb.getInstance().toggleMidQuickRelease(false);
         //Climb.getInstance().toggleHighQuickRelease(false);
        VisionManager.getInstance().toggleLimelight(true);
        //climb.setClimbState(ClimbState.JOG);
        //climbRoutine = null;
        //routine = new ClimbRoutine();
        LED.getInstance().setRainbow();

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
/*
    if(operator.getRawButtonPressed(1)){
        if(climbRoutine==null||climbRoutine.isInterrupted()){
             climbRoutine = new Thread(routine);
             climbRoutine.start();
        }
         else
             climbRoutine.resume();  
     }
      else if(operator.getRawButtonReleased(1)){
          if(climbRoutine!=null)
             climbRoutine.suspend();
    }
    */
    if(driver.getLeftBumper())
      drive.resetHeading();

     if(driver.getRawAxis(2)>.2)
       intake.setIntakeState(IntakeState.RUN_ALL);
     else if(driver.getRightBumper())
      intake.setIntakeState(IntakeState.INDEX_REVERSE);
    else if(driver.getAButton())
        Intake.getInstance().setIntakeState(IntakeState.SHOOTING);
    else if(driver.getBButton())
        Intake.getInstance().setIntakeState(IntakeState.SPIT_BALL);
    else if(operator.getRawAxis(2)>.2)
        Intake.getInstance().setIntakeState(IntakeState.INTAKE_REVERSE);
    else
      intake.setOff();


    if(driver.getYButton())
        intake.toggleIntake();
  
    
    if(driver.getRawAxis(3)>.2)
        intake.index();


    if(driver.getXButtonPressed())
      drive.setDriveState(DriveState.VISION);

   



}


    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        killAuto();
        enabled.setBoolean(false);
        //if(climbRoutine!=null)
        //climbRoutine.interrupt();
        //climbRoutine = null;  
        VisionManager.getInstance().toggleLimelight(false);
     }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        startSubsystems();
        //Climb.getInstance().setJog();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        /*
        if(operator.getRawButtonPressed(5)) 
            //climb.toggleMidQuickRelease(true);
        else if(operator.getRawButtonPressed(1))
             //climb.toggleMidQuickRelease(false);
        else if(operator.getRawButtonPressed(6))
            //climb.toggleHighQuickRelease(true);
        else if(operator.getRawButtonPressed(2))
            //climb.toggleHighQuickRelease(false);
            */
    }

    private void startSubsystems() {
        odometry.start();
        drive.start();
        intake.start();
        //vision.start();
        //climb.start();

    }

    public synchronized void killAuto() {
        if (selectedAuto != null&&autoThread!=null) {
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
            drive.stopMotors();
            drive.setTeleop();
        }
    }

    @Override
    public void simulationInit() {
        ClassInformationSender.updateReflectionInformation(
                new File("C:/Users/60002/AppData/Roaming/AutoBuilder"+ "/robotCodeData.json"));
    }
}