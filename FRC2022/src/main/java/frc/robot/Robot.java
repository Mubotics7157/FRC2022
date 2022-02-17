// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.Subystem.Drive;
import frc.Subystem.RobotTracker;
import frc.Subystem.Shooter;
import frc.util.Threading.ThreadScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * project.
 */
public class Robot extends TimedRobot {
  public static Joystick leftStick = new Joystick(1);
  public static Joystick rightStick = new Joystick(1);
  public static final XboxController operator = new XboxController(0);
  Drive drive = Drive.getInstance();
  RobotTracker tracker = RobotTracker.getInstance();
  ExecutorService executor = Executors.newFixedThreadPool(2); 
  ThreadScheduler scheduler = new ThreadScheduler();
  Shooter shooter = new Shooter();
  
  
@Override
public void robotInit() {
    drive.setPeriod(Duration.ofMillis(20));
    tracker.setPeriod(Duration.ofMillis(5));
    scheduler.schedule(drive, executor);
    scheduler.schedule(tracker, executor);
}
  @Override
  public void robotPeriodic() {
  }
  @Override
  public void teleopInit() {
    scheduler.resume();
    drive.setTeleop();
  }
@Override
public void teleopPeriodic() {
  if(operator.getRawButtonPressed(1)){
    //^^ A button
      //topSpeed -= .05;
      shooter.adjustShooterSpeeds(-.05, -.05);
    
    }
    if(operator.getRawButtonPressed(2)){
    //^^ B button
      //topSpeed += .05;
      shooter.adjustShooterSpeeds(.05, .05);
    
    }
    if(operator.getRawButtonPressed(3)){
    //^^ X button
      //bottomSpeed -= .05;
      shooter.adjustShooterSpeeds(-.05, -.05);
      
    }
    if(operator.getRawButtonPressed(4)){
    //^^ Y button
      //bottomSpeed += .05;
      shooter.adjustShooterSpeeds(.05, .05);
      
    }
    

  if(operator.getRawButtonPressed(6)){
    Shooter.adjustShooterSpeeds(topSpeed +=.05, bottomSpeed +=.05);
  }
      //double throttle = -m_gamepad.getY(Hand.kLeft);
      //drive.tankDriveVelocity(throttle*10,throttle*10);
}

@Override
public void simulationPeriodic() {
}
}