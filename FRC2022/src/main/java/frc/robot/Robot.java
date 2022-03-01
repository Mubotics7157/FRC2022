
package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.Serializer;
import frc.Subystem.SwerveDrive.SwerveDrive;
import frc.Subystem.SwerveDrive.SwerveTracker;
import frc.auto.AutoRoutine;
import frc.auto.AutoRoutineGenerator;
//import frc.auto.AutoRoutineGenerator;
import frc.util.Threading.ThreadScheduler;

public class Robot extends TimedRobot {
  public static final XboxController driver = new XboxController(0);
  public static final XboxController operator = new XboxController(1);
  SwerveDrive swerve = SwerveDrive.getInstance(); 
  SwerveTracker tracker = SwerveTracker.getInstance();
  //VisionManager visionManager = VisionManager.getInstance();

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
    scheduler.schedule(serializer, executor);
    scheduler.schedule(swerve, executor);
    scheduler.schedule(tracker, executor);
}
  @Override
  public void robotPeriodic() {
  }

@Override
public void autonomousInit() {
  scheduler.resume();
    AutoRoutine option = new AutoRoutineGenerator().TestPath();
    auto = new Thread(option);
    auto.start();
}

@Override
public void autonomousPeriodic() {
  SmartDashboard.putBoolean("auto", auto.isInterrupted());
}

  public void teleopInit() {
    if(auto!=null)
      auto.interrupt();
    scheduler.resume();
    SwerveDrive.getInstance().setRobotOriented();
    serializer.setOff();
    compressor.enableDigital();
    SmartDashboard.putNumber("turning d ", .02);
    SmartDashboard.putNumber("turning p ", .02);
    SmartDashboard.putNumber("LL P", 0);
    swerve.setFieldOriented();
  }

public void teleopPeriodic() {
      if(driver.getRawAxis(2)>.2)
        serializer.setAll();  
      else if(driver.getRawButton(6))
        serializer.setShooting();
      else if(driver.getRawAxis(3)>.2)
        serializer.setEjecting();
      else if(driver.getRawButton(5))
        serializer.setIndexing();
      else
        serializer.setOff();

      if(driver.getRawButton(4)){
        serializer.toggleIntake(false);
      }
      else if(driver.getRawButton(2))
        serializer.toggleIntake(true);
      
      
        /*
      if(driver.getRawButton(3))
        climb.setExtending();
      else if(driver.getRawButton(1))
        climb.setRetracting();
        */

        if(operator.getRawButtonPressed(1))
          swerve.setTargetAlign();
        else if(operator.getRawButton(3))
          swerve.setFieldOriented();

      if(operator.getRawButtonPressed(2)){
        swerve.increaseLLP();
      }
      else if(operator.getRawButton(5))
      swerve.zeroYaw();
      else if(operator.getRawButton(4))
      tracker.resetOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
      
      
      
      
      
      

}

@Override
public void testInit() {
  scheduler.resume();
}
} 