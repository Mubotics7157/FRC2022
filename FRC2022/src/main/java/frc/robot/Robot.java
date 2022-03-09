
package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
//import frc.auto.AutoRoutineGenerator;
import frc.util.Threading.ThreadScheduler;

public class Robot extends TimedRobot {
  public static Joystick driver = new Joystick(0);
  public static final XboxController operator = new XboxController(1);
  SwerveDrive swerve = SwerveDrive.getInstance(); 
  SwerveTracker tracker = SwerveTracker.getInstance();
  VisionManager visionManager = VisionManager.getInstance();
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
    tracker.setPeriod(Duration.ofMillis(5));
    climb.setPeriod(Duration.ofMillis(40));
    visionManager.setPeriod(Duration.ofMillis(39));
    scheduler.schedule(serializer, executor);
    scheduler.schedule(swerve, executor);
    scheduler.schedule(tracker, executor);
    scheduler.schedule(climb, executor);
    scheduler.schedule(visionManager, executor);
}
  @Override
  public void robotPeriodic() {
  }

@Override
public void autonomousInit() {
  scheduler.resume();
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
      if(driver.getRawAxis(2)>.2)
        serializer.setAll();  
      else if(driver.getRawButton(6))
        serializer.setShooting();
      else if(driver.getRawButton(5))
        serializer.setIndexing();
      else
        serializer.setOff();

       if(driver.getRawButton(2))
        serializer.toggleIntake(true);
       else if(driver.getRawButton(2))
        serializer.toggleIntake(false);
        if(operator.getRawButtonPressed(1))
          swerve.setTargetAlign();
        else if(operator.getRawButton(3))
          swerve.setFieldOriented();
      else if(operator.getRawAxis(2)>0)
        swerve.setRobotOriented();
      if(operator.getRawButtonPressed(2)){
        swerve.zeroYaw();
      }
      
      
      
      
      

}

@Override
public void testInit() {
  scheduler.resume();
}
} 