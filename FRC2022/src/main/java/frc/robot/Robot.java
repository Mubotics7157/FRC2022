package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.Subystem.SwerveDrive.SwerveDrive;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.util.Threading.ThreadScheduler;

public class Robot extends TimedRobot {
  public static final XboxController operator = new XboxController(0);
  SwerveDrive swerve = SwerveDrive.getInstance();
  SwerveTracker tracker = SwerveTracker.getInstance();
  ExecutorService executor = Executors.newFixedThreadPool(2); 
  ThreadScheduler scheduler = new ThreadScheduler();
  
  @Override
  public void robotInit() {
    tracker.setPeriod(Duration.ofMillis(5));
    swerve.setPeriod(Duration.ofMillis(20));
    scheduler.schedule(swerve, executor);
    scheduler.schedule(tracker, executor);
  }
  @Override
  public void robotPeriodic() {
  }
  @Override
  public void teleopInit() {
    scheduler.resume();
    swerve.setFieldOriented();
  }
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}