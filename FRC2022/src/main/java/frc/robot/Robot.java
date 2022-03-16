
package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Subystem.Climb;
import frc.Subystem.LED;
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
  LED led = new LED();

  ExecutorService executor = Executors.newFixedThreadPool(2); 
  ThreadScheduler scheduler = new ThreadScheduler();
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  Thread auto;
  SendableChooser<AutoRoutine> sendableChooser = new SendableChooser<AutoRoutine>();

  
  Serializer serializer = Serializer.getInstance();
  
@Override
public void robotInit() {
    serializer.setPeriod(Duration.ofMillis(20));
    swerve.setPeriod(Duration.ofMillis(20));
    tracker.setPeriod(Duration.ofMillis(30));
    climb.setPeriod(Duration.ofMillis(30));
    vision.setPeriod(Duration.ofMillis(30));
    scheduler.schedule(vision, executor);
    scheduler.schedule(serializer, executor);
    scheduler.schedule(swerve, executor);
    scheduler.schedule(tracker, executor);
    scheduler.schedule(climb, executor);
    //new Thread(scheduler).start();
    swerve.calibrateGyro();
    swerve.resetGyro();
    sendableChooser.addOption("two ball", AutoRoutineGenerator.TwoBallAuto());
    sendableChooser.setDefaultOption("default", AutoRoutineGenerator.oneBallAuto());
    SmartDashboard.putData(sendableChooser);

    /*new Thread(new Runnable() {
      public void run(){
        serializer.update();
        try{
          Thread.sleep(20);
        } catch (InterruptedException e){
          return;
        }
        
      }
    }).start();*/
scheduler.resume();
serializer.setOff();
  }
  @Override
  public void robotPeriodic() {
  }

@Override
public void autonomousInit() {
  
  AutoRoutine option = AutoRoutineGenerator.TwoBallAuto();
  auto = new Thread(option);
  auto.start();
  scheduler.resume();
  climb.setOff();
  
}

@Override
public void autonomousPeriodic() {
  //System.out.print();
}


  public void teleopInit() {
    led.isAligning = false;
    if(auto!=null)
      auto.interrupt();
    scheduler.resume();
    compressor.enableDigital();
    SwerveDrive.getInstance().setFieldOriented();
   Serializer.getInstance().setOff();
    VisionManager.getInstance().setOn();

    
    serializer.setShooterSpeed(1350, 1350*1.08);
  }

  
public void teleopPeriodic() {

    //intake states
    if(driver.getRawAxis(2)>.2)
      serializer.setAll();  
    else if(driver.getRawButton(1))
      serializer.setShooting();
    else if(driver.getRawButton(6))
      serializer.setIndexing();
    else if(operator.getRawButton(5))
      serializer.setIntaking();
    else
      serializer.setOff();

    if(driver.getRawAxis(3)>.2)
      serializer.runIndexer();


    //extend and retract the intake
    if(driver.getRawButton(4))
      serializer.toggleIntake(false);
    else if(driver.getRawButton(2)||operator.getRawButton(4))
      serializer.toggleIntake(true);

    //setting the modes for the swerve drive
    if(driver.getRawButtonPressed(3)){
        swerve.setTargetAlign();
        led.isAligning = true;
    }
      else if(operator.getRawButtonPressed(3)){
        swerve.setFieldOriented();
        led.isAligning = false;
      }
      else if(operator.getRawButton(11)){
        swerve.setRobotOriented();
        led.isAligning = false;
      }
    //toggle the vision on and off
    if(operator.getRawButtonPressed(12))
      vision.setOff();
    else if(operator.getRawButtonPressed(6))
      vision.setOn();
    
    if(driver.getRawButtonPressed(5))
      swerve.zeroYaw();
    
    
    if(operator.getRawButtonPressed(7))
      climb.setManual();
      else if (operator.getRawButtonPressed(10))
        climb.setOff();
    
      

}

@Override
public void testInit() {
  scheduler.resume();
  climb.setHoming();
}
@Override
public void testPeriodic() {
  //swerve.goToZero();
}
} 