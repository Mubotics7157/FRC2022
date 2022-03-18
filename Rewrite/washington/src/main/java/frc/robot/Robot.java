package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subsystem.Drive;
import frc.Subsystem.Intake;
import frc.Subsystem.Odometry;
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


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    tracker.start();
    swerve.start();
    intake.start();
    swerve.calibrateGyro();
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
  }

  @Override
  public void teleopPeriodic() {
    if(driver.getLeftBumper())
      swerve.resetHeading();

    if(driver.getRawAxis(3)>.2)
      intake.setIntakeState(IntakeState.INDEX);
    else if(driver.getRawAxis(2)>.2)
      intake.setIntakeState(IntakeState.RUN_ALL);
    else if(driver.getRightBumper())
      intake.setIntakeState(IntakeState.INDEX_REVERSE);
    else if(driver.getAButton())
      intake.setIntakeState(IntakeState.SHOOTING);
    else
      intake.setIntakeState(IntakeState.OFF);
    
  
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
