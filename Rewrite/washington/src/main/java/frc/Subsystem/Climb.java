package frc.Subsystem;

import java.util.ResourceBundle.Control;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.AbstractSubsystem;
import frc.util.OrangeUtility;

public class Climb extends AbstractSubsystem {
    DoubleSolenoid midQuickRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    DoubleSolenoid highQuickRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  
    //DigitalInput midMagSensor = new DigitalInput(3);  
    //DigitalInput highMagSensor = new DigitalInput(4);
    //PlZ CHANGE DIGITAL INPUT PORT ONCE ITS FOUND :P
  
    TalonFX midClimb = new TalonFX(29);
    TalonFX highClimb = new TalonFX(40);
    //PLZ CHANGE ID FOR HIGH CLIMB ONCE ITS FOUND :P

    private static Climb instance = new Climb();

    public static Climb getInstance(){
        return instance;
    }

    private Climb(){
        super(30,30);
        OrangeUtility.sleep(1000);
        midClimb.configFactoryDefault();
        
        midClimb.config_kP(0, .5);
        midClimb.config_kD(0, .3);
        midClimb.configNeutralDeadband(.2);
        midClimb.setInverted(true);
        midClimb.setNeutralMode(NeutralMode.Brake);
        midClimb.setSelectedSensorPosition(-590000);

        //highClimb.configForwardSoftLimitEnable(true);
        //highClimb.configReverseSoftLimitEnable(true);
        //highClimb.configReverseSoftLimitThreshold(-1468417);//-1302192
        //highClimb.configForwardSoftLimitThreshold(5000);
        highClimb.configFactoryDefault();

        highClimb.config_kP(0, .5);
        highClimb.config_kD(0, .05);
        highClimb.configNeutralDeadband(.2);
        highClimb.setInverted(true);
        highClimb.setNeutralMode(NeutralMode.Brake);
        highClimb.setSelectedSensorPosition(0);
        //PLZ CHAGE SENSOR POSITION ONCE ITS FOUND :P
    }


    public synchronized void climbRoutine(){
        if((Robot.driver.getRightBumper() || Robot.operator.getRawButton(7)) && midQuickRelease.get() != Value.kReverse){
          midQuickRelease.set(Value.kReverse);
        }
        

        else if((Robot.driver.getRightBumper() || Robot.operator.getRawButton(7)) && midQuickRelease.get() ==Value.kReverse){
          midClimb.set(ControlMode.Position, 160000);


        }
      }

    public synchronized void manualMidClimb(){
        midClimb.set(ControlMode.PercentOutput, -Robot.operator.getRawAxis(1));
    }

    public synchronized void manualHighClimb(){
        highClimb.set(ControlMode.PercentOutput, -Robot.operator.getRawAxis(5));
    }
    
    public synchronized void resetMidClimb(){
        /*if(midMagSensor.get() && midQuickRelease.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 590000) > 1000)
          midClimb.set(ControlMode.Position, 590000);
        else if(!midMagSensor.get() && midQuickRelease.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 590000) < 1000){
          midQuickRelease.set(Value.kForward);
            }*/
    }

    public synchronized void resetHighClimb(){
     /* if(highMagSensor.get() && highQuickRelease.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 590000) > 1000)
        highClimb.set(ControlMode.Position, 590000);
      else if(!highMagSensor.get() && highQuickRelease.get() == Value.kReverse && Math.abs(highClimb.getSelectedSensorPosition() - 590000) < 1000){
        highQuickRelease.set(Value.kForward);
      }*/
    }


    public synchronized void zeroMidClimb(){
        /*if(midMagSensor.get())
        midClimb.set(ControlMode.PercentOutput, -0.4);
        else if(!midMagSensor.get()){
        midClimb.set(ControlMode.PercentOutput, 0);
        midClimb.setSelectedSensorPosition(0);
        }*/
    }

    public synchronized void zeroHighClimb(){
      /*if(highMagSensor.get())
      highClimb.set(ControlMode.PercentOutput, -0.4);
      else if(!highMagSensor.get()){
        highClimb.set(ControlMode.PercentOutput, 0);
        highClimb.setSelectedSensorPosition(0);
      }*/
    }

    public synchronized void toggleHighSolenoid(){
      
      if(highQuickRelease.get() != Value.kForward) {
        highQuickRelease.set(Value.kForward);
        //SmartDashboard.putBoolean("is forward?", true);

      } else if(highQuickRelease.get() != Value.kReverse) {
        highQuickRelease.set(Value.kReverse);
        //SmartDashboard.putBoolean("is Reverse?", true);
      } else{
        highQuickRelease.set(Value.kReverse);
        //SmartDashboard.putBoolean("is Reverse 2?", false);
      }
      
        //highQuickRelease.set(Value.kForward);
    }

    public synchronized void toggleMidSolenoid(){
        if(midQuickRelease.get() != Value.kForward)
          midQuickRelease.set(Value.kForward);
        else if(midQuickRelease.get() != Value.kReverse)
          midQuickRelease.set(Value.kReverse);
          else
          midQuickRelease.set(Value.kReverse);
      }
    
    public synchronized void sethighForward(){
      highQuickRelease.set(Value.kForward);
    }

    public synchronized void setHighReverse(){
      highQuickRelease.set(Value.kReverse);
    }


    @Override
    public void logData() {
        SmartDashboard.putString("high quick release", highQuickRelease.get().toString());
        //SmartDashboard.putBoolean("high magnet detected?", highMagSensor.get());
        SmartDashboard.putNumber("high climb encoder", highClimb.getSelectedSensorPosition());

        SmartDashboard.putString("mid quick release", midQuickRelease.get().toString());
        //SmartDashboard.putBoolean("mid magnet detected?", midMagSensor.get());
        SmartDashboard.putNumber("mid climb encoder", midClimb.getSelectedSensorPosition());
    }

    @Override
    public void selfTest() {
   

        
    }

    
}