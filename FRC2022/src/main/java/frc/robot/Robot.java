package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.Climb;
import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;
import frc.Subystem.SwerveDrive.SwerveTracker;
//import frc.auto.AutoRoutineGenerator;
import frc.util.Threading.ThreadScheduler;

public class Robot extends TimedRobot {
  public static final XboxController operator = new XboxController(0);
  public static final XboxController operator2 = new XboxController(1);
  SwerveDrive swerve = SwerveDrive.getInstance(); 
  SwerveTracker tracker = SwerveTracker.getInstance();
  //VisionManager visionManager = VisionManager.getInstance();

  ExecutorService executor = Executors.newFixedThreadPool(2); 
  ThreadScheduler scheduler = new ThreadScheduler();
  //Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  Thread auto;

  
  Serializer serializer = Serializer.getInstance();
  
@Override
public void robotInit() {
    serializer.setPeriod(Duration.ofMillis(20));
    swerve.setPeriod(Duration.ofMillis(20));
    //drive.setPeriod(Duration.ofMillis(20));
    //vision.setPeriod(Duration.ofMillis(5));
    tracker.setPeriod(Duration.ofMillis(5));
    scheduler.schedule(serializer, executor);
    scheduler.schedule(swerve, executor);
    scheduler.schedule(tracker, executor);
    //scheduler.schedule(drive, executor);
    //scheduler.schedule(vision, executor);
}
  @Override
  public void robotPeriodic() {
  }

@Override
public void autonomousInit() {
  scheduler.resume();
    //AutoRoutine option = new AutoRoutineGenerator().niceSideBlue5BallRoutine();
    //auto = new Thread(option);
    //auto.start();
}

@Override
public void autonomousPeriodic() {
  SmartDashboard.putBoolean("auto", auto.isInterrupted());
}

  public void teleopInit() {
    if(auto!=null)
      auto.interrupt();
    scheduler.resume();
    /*if(RobotController.isBrownedOut())
      SwerveDrive.getInstance().setRobotOriented();
    else
      SwerveDrive.getInstance().setFieldOriented();
      */
    SwerveDrive.getInstance().setFieldOriented();
    //SwerveDrive.getInstance().setRobotOriented();
    serializer.setOff();
  }

public void teleopPeriodic() {
      if(operator.getRawAxis(2)>.2)
        serializer.setAll();  
      else if(operator.getRawButton(6))
        serializer.setShooting();
      else if(operator.getRawAxis(3)>.2)
        serializer.setEjecting();
      else if(operator.getRawButton(5))
        serializer.setIndexing();
      else if(operator.getRawButton(3))
        serializer.setClimb(true);
      else if(operator.getRawButton(1))
        serializer.setClimb(false);
      else
        serializer.setOff();

      if(operator.getRawButton(4)){
        serializer.toggleIntake(false);
      }
      else if(operator.getRawButton(2))
        serializer.toggleIntake(true);

}


} 