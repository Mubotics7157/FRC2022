
package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.Climb;
import frc.Subystem.Serializer;
import frc.Subystem.VisionManager;  
import frc.Subystem.SwerveDrive.SwerveDrive;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.auto.AutoRoutine;
import frc.auto.AutoRoutineGenerator;
import frc.util.Threading.ThreadScheduler;

public class Robot extends TimedRobot {
  public static Joystick driver = new Joystick(0);
  public static final XboxController operator = new XboxController(1);
  SwerveDrive swerve = SwerveDrive.getInstance(); 
  SwerveTracker tracker = SwerveTracker.getInstance();
  VisionManager vision = VisionManager.getInstance();
  Climb climb = new Climb();

  ExecutorService executor = Executors.newFixedThreadPool(2); 
  ThreadScheduler scheduler = new ThreadScheduler();
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  Thread auto;

  
  Serializer serializer = Serializer.getInstance();
  
@Override
public void robotInit() {
    serializer.setPeriod(Duration.ofMillis(20));
    swerve.setPeriod(Duration.ofMillis(20));
    tracker.setPeriod(Duration.ofMillis(30));
    climb.setPeriod(Duration.ofMillis(50));
    vision.setPeriod(Duration.ofMillis(30));
    scheduler.schedule(vision, executor);
    scheduler.schedule(serializer, executor);
    scheduler.schedule(swerve, executor);
    scheduler.schedule(tracker, executor);
    scheduler.schedule(climb, executor);
    swerve.calibrateGyro();
    swerve.resetGyro();
}
  @Override
  public void robotPeriodic() {
  }

@Override
public void autonomousInit() {
  scheduler.resume();
  AutoRoutine option = AutoRoutineGenerator.TwoBallAuto();
  auto = new Thread(option);
  
}

@Override
public void autonomousPeriodic() {
  SmartDashboard.putBoolean("auto", auto.isInterrupted());
}

  public void teleopInit() {
  if(auto!=null)
      auto.interrupt();
    scheduler.resume();
    serializer.setOff();
    compressor.enableDigital();
    swerve.setFieldOriented();
  }

  
public void teleopPeriodic() {

    //intake states
    if(driver.getRawAxis(2)>.2)
      serializer.setAll();  
    else if(operator.getRawButton(2))
      serializer.setShooting();
    else if(driver.getRawButton(6))
      serializer.setIndexing();
    else
      serializer.setOff();

    if(driver.getRawAxis(3)>.2)
      serializer.runIndexer();


    //extend and retract the intake
    if(driver.getRawButton(4))
      serializer.toggleIntake(false);
    else if(driver.getRawButton(2))
      serializer.toggleIntake(true);

    //setting the modes for the swerve drive
    /*
    if(operator.getRawButtonPressed(1))
        swerve.setTargetAlign();
      else if(operator.getRawButtonPressed(3))
        swerve.setFieldOriented();
      else if(operator.getRawButton(12))
        swerve.setRobotOriented();
*/
    //toggle the vision on and off
    if(operator.getRawButtonPressed(4))
      vision.setOff();
    else if(operator.getRawButtonPressed(6))
      vision.setOn();
    
    if(driver.getRawButtonPressed(5))
      swerve.zeroYaw();
    
    //serializer.adjustShooterSpeeds(operator.getRawAxis(3)*-200, operator.getRawAxis(3)*-200/1.08);

    if(operator.getRawButtonPressed(11))
      climb.toggleClimbSolenoid();
    
    if(operator.getRawButtonPressed(7))
      climb.setManual();
      

}

@Override
public void testInit() {
  scheduler.resume();
}
@Override
public void testPeriodic() {
  //swerve.goToZero();
}
} 