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
import frc.Subystem.Serializer;
import frc.Subystem.VisionManager;
import frc.util.Threading.ThreadScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * project.
 */
public class Robot extends TimedRobot {
  //public static Joystick leftStick = new Joystick(1);
  //public static Joystick rightStick = new Joystick(1);
  public static final XboxController operator = new XboxController(0);
  ExecutorService executor = Executors.newFixedThreadPool(2); 
  ThreadScheduler scheduler = new ThreadScheduler();
  Drive drive = Drive.getInstance();
  Serializer serializer = Serializer.getInstance();
  VisionManager vision = VisionManager.getInstance();
  
@Override
public void robotInit() {
    serializer.setPeriod(Duration.ofMillis(20));
    drive.setPeriod(Duration.ofMillis(20));
    vision.setPeriod(Duration.ofMillis(5));
    scheduler.schedule(serializer, executor);
    scheduler.schedule(drive, executor);
    scheduler.schedule(vision, executor);
}
  @Override
  public void robotPeriodic() {
  }
  @Override
  public void teleopInit() {
    scheduler.resume();
    serializer.setOff();
    Drive.getInstance().setTeleop();
  }
@Override
public void teleopPeriodic() {
      if(operator.getRawAxis(3)>.2)
        serializer.runAll();
      else if(operator.getRawAxis(2)>.2)
        serializer.setIntaking();
      else if(operator.getRawButton(5))
        serializer.setIndexing();
      else if(operator.getRawButton(6))
        serializer.setShooting();
      else if(operator.getRawButton(1))
        serializer.setEjecting();
      else
        serializer.setOff();

      
      //double throttle = -m_gamepad.getY(Hand.kLeft);
      //drive.tankDriveVelocity(throttle*10,throttle*10);
}

@Override
public void simulationPeriodic() {
}
}