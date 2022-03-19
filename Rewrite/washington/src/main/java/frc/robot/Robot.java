package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subsystem.Climb;
import frc.Subsystem.Drive;
import frc.Subsystem.Intake;
import frc.Subsystem.Odometry;
import frc.Subsystem.VisionManager;
import frc.Subsystem.Drive.DriveState;
import frc.Subsystem.Intake.IntakeState;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static XboxController driver = new XboxController(0);
  public static Joystick operator = new Joystick(1);
  Drive swerve = Drive.getInstance();
  Odometry tracker = Odometry.getInstance();
  Intake intake = Intake.getInstance();
  VisionManager vision = VisionManager.getInstance();
  Climb climb = Climb.getInstance();

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    tracker.start();
    swerve.start();
    intake.start();
    vision.start();
    climb.start();
    swerve.calibrateGyro();
       SmartDashboard.putNumber("shooter ratio", 1);
       SmartDashboard.putNumber("top wheel setpoint", 1000);
       SmartDashboard.putNumber("bot wheel setpoint", 1000);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
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
    swerve.resetHeading();
    swerve.setDriveState(DriveState.FIELD_ORIENTED);
    compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() {
    if(driver.getLeftBumper())
      swerve.resetHeading();

    if(driver.getRawAxis(2)>.2)
      intake.setIntakeState(IntakeState.RUN_ALL);
    else if(driver.getRightBumper())
      intake.setIntakeState(IntakeState.INDEX_REVERSE);
    else if(driver.getAButton())
      intake.setIntakeState(IntakeState.SHOOTING);
    else
      intake.setOff();
    

    if(driver.getYButton())
      intake.toggleIntake(true);
    else if (driver.getBButton())
      intake.toggleIntake(false);
  

    if(operator.getRawButtonPressed(3)){
      Intake.getInstance().setShooterSpeeds();
      Intake.getInstance().setShooterRatio();
    }

    if(driver.getXButtonPressed())
      swerve.setDriveState(DriveState.VISION);
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
