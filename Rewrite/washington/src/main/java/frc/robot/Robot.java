package frc.robot;

import java.io.File;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subsystem.Climb;
import frc.Subsystem.Drive;
import frc.Subsystem.Intake;
import frc.Subsystem.Odometry;
import frc.Subsystem.Shooter;
import frc.Subsystem.VisionManager;
import frc.Subsystem.Drive.DriveState;
import frc.Subsystem.Intake.IntakeState;
import frc.Subsystem.Shooter.ShooterMode;
import frc.auton.FiveBall;
import frc.auton.TemplateAuto;
import frc.auton.TwoBall;
import frc.auton.WeakSide;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.reflection.ClassInformationSender;
import frc.util.OrangeUtility;


public class Robot extends TimedRobot {

    public static XboxController driver = new XboxController(0);
    public static Joystick operator = new Joystick(1);

    //Subsystems
    private final Odometry odometry = Odometry.getInstance();
    private final Drive drive = Drive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final VisionManager vision = VisionManager.getInstance();
    private final Climb climb = Climb.getInstance();

    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final SendableChooser<String> sideChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        OrangeUtility.sleep(1500);

        SmartDashboard.putNumber("top wheel setpoint", 1000);
        SmartDashboard.putNumber("shooter ratio", 1);
        SmartDashboard.putNumber("shot adjustment", 1);
        SmartDashboard.putNumber("flywheel kP",.01);
        


        startSubsystems();
        drive.resetHeading();
        OrangeUtility.sleep(50);
        odometry.setOdometry(new Pose2d());
        VisionManager.getInstance().toggleLimelight(false);

        AutonomousContainer.getInstance().initialize(
                true,
                new CommandTranslator(
                        drive::setAutoPath, //The consumer to call to set the new trajectory
                        drive::stopMotors, //The runnable to call to stop the robot from moving
                        drive::setAutoRotation, //The consumer to call to set the autonomous rotation (can be null is the robot is not holonomic)
                        drive::isFinished, //The boolean supplier to call to check if the trajectory is done. This lambada should return false until the path has been fully (and is within error of the final position/rotation) driven.
                        drive::getAutoTime, //The double supplier to call to get the elapsed time of the trajectory. This lambada must return 0.0 immediately after a new trajectory is set and should return the elapsed time of the current trajectory that is being driven.
                        odometry::resetPosition, //The consumer to call to set the initial pose of the robot at the start of autonomous
                        false //Whether to run the commands on the main thread. If this is true, the commands will be run on the main thread. If this is false, the commands will be run on the autonomous thread. If you are unsure, it is safer to leave this as true. If you've designed your robot code to be thread safe, you can set this to false. It will allow the methods you call to be blocking which can simplify some code.
                ), 
                false, //crashOnError – Should the robot crash on error? If this is enabled, and an auto fails to load, the robot will crash. If this is disabled, the robot will skip the invalid auto and continue to the next one.
                null //timedRobot – The timed robot (should be able to just use the 'this' keyword) to use to create the period function for the autos. This can be null if you're running autos asynchronously.
        );

        AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));
        sideChooser.setDefaultOption("Blue", "blue"); 
        sideChooser.addOption("Red", "red");

        SmartDashboard.putData("Auto choices", autoChooser);
        SmartDashboard.putData("Red or Blue", sideChooser); 

    }
    
    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {

        String selectedAuto = autoChooser.getSelected();

        if(selectedAuto == null)
            selectedAuto = "twoBall";

        VisionManager.getInstance().toggleLimelight(true);
        shooter.setStatic();
        AutonomousContainer.getInstance().runAutonomous(selectedAuto, sideChooser.getSelected(), true);

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
        
        startSubsystems();
        compressor.enableDigital();
        drive.setDriveState(DriveState.TELE);
        drive.resetHeading();
        drive.setDriveState(DriveState.TELE);
        compressor.enableDigital();

        VisionManager.getInstance().toggleLimelight(true);

        shooter.setInterpolating();
        
        //climb.setForward();

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    if(driver.getLeftBumper())
      drive.resetHeading();

     if(driver.getRawAxis(2)>.2)
       intake.setIntakeState(IntakeState.RUN_ALL);
 
    else if(driver.getRawButton(9)){
        Intake.getInstance().setIntakeState(IntakeState.INTAKE_REVERSE);
    }
    else if(driver.getRawAxis(3)>.2)// && shooter.atSpeed())
        intake.setIntakeState(IntakeState.INDEX);
    else
      Intake.getInstance().setIntakeState(IntakeState.OFF);


   if(operator.getRawButtonPressed(1)){
       shooter.setInterpolating();
    }
    else if(operator.getRawButtonPressed(2)){
        shooter.setStatic();
    }

    if(driver.getRawButtonPressed(10))
        intake.toggleIntake();

    if(driver.getXButtonPressed())
      drive.setDriveState(DriveState.VISION);
   
    
      if(operator.getRawAxis(2) > 0.5)
        climb.zeroClimb();
      else if(operator.getRawAxis(3) > 0.5)
        climb.resetClimb();
    else if(driver.getRightBumper() || operator.getRawButton(7))
    climb.climbRoutine();
      else
        climb.manualClimb();

     if(operator.getRawButtonPressed(5))
        climb.toggleClimbSolenoid();
    else if(operator.getRawButtonPressed(6))
        climb.toggleHighSolenoid();
}


    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
       
    

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
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

    }

    private void startSubsystems() {
        odometry.start();
        drive.start();
        intake.start();
        vision.start();
        shooter.start();
        climb.start();

    }



    @Override
    public void simulationInit() {
        com.dacubeking.AutoBuilder.robot.reflection.ClassInformationSender.updateReflectionInformation("frc");
    }
}
