package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.VisionManager;
import frc.Subystem.SwerveDrive.SwerveDrive;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.auto.AutoRoutine;
import frc.auto.AutoRoutineGenerator;
import frc.util.Threading.ThreadScheduler;

public class Robot extends TimedRobot {
  public static final XboxController operator = new XboxController(0);
  SwerveDrive swerve = SwerveDrive.getInstance();
  SwerveTracker tracker = SwerveTracker.getInstance();
  VisionManager visionManager = VisionManager.getInstance();

  ExecutorService executor = Executors.newFixedThreadPool(2); 
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;
  
  @Override
  public void robotInit() {
    tracker.setPeriod(Duration.ofMillis(5));
    swerve.setPeriod(Duration.ofMillis(20));
    visionManager.setPeriod(Duration.ofMillis(5));
    scheduler.schedule(swerve, executor);
    scheduler.schedule(tracker, executor);
    scheduler.schedule(visionManager, executor);
  }
  @Override
  public void robotPeriodic() {
  }

@Override
public void autonomousInit() {
  scheduler.resume();
    AutoRoutine option = new AutoRoutineGenerator().niceSideBlue5BallRoutine();
    auto = new Thread(option);
    auto.start();
}

@Override
public void autonomousPeriodic() {
  SmartDashboard.putBoolean("auto", auto.isInterrupted());
}
  @Override
  public void teleopInit() {
    if(auto!=null)
      auto.interrupt();
    scheduler.resume();
    if(RobotController.isBrownedOut())
      SwerveDrive.getInstance().setRobotOriented();
    else
      SwerveDrive.getInstance().setFieldOriented();
  }
  @Override
  public void teleopPeriodic() {
    if(operator.getRawButtonPressed(1))
      SwerveDrive.getInstance().setTargetAlign();
  }

  @Override
  public void simulationPeriodic() {
  }
}