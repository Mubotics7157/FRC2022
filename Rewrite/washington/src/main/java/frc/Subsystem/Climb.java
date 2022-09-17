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

public class Climb extends AbstractSubsystem {
    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    DoubleSolenoid midQuickRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  
    DigitalInput magSensor = new DigitalInput(3);  
  
    TalonFX midClimb = new TalonFX(29);

    private static Climb instance = new Climb();

    public static Climb getInstance(){
        return instance;
    }

    private Climb(){
        super(30,30);
        midClimb.setSelectedSensorPosition(0);
        midClimb.configFactoryDefault();
        midClimb.config_kP(0, .5);
        midClimb.config_kD(0, .3);
        midClimb.configNeutralDeadband(.2);
        midClimb.setInverted(true);
        midClimb.setNeutralMode(NeutralMode.Brake);
    }


    public synchronized void climbRoutine(int button){
        if(Robot.operator.getRawButton(button) && intakeSolenoid.get() != Value.kReverse)
        intakeSolenoid.set(Value.kReverse);
        else if(Robot.operator.getRawButton(button) && intakeSolenoid.get() ==Value.kReverse)
        midClimb.set(ControlMode.Position, 160000);
      }

    public synchronized void manualClimb(){
        midClimb.set(ControlMode.PercentOutput, Robot.operator.getRawAxis(1) * 0.4);
    }
    
      public synchronized void resetClimb(){
        if(magSensor.get() && intakeSolenoid.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 600000) > 1000)
          midClimb.set(ControlMode.Position, 600000);
        else if(!magSensor.get() && intakeSolenoid.get() == Value.kReverse && Math.abs(midClimb.getSelectedSensorPosition() - 600000) < 1000){
          intakeSolenoid.set(Value.kForward);
            }
        }
    public synchronized void zeroClimb(){
        if(magSensor.get())
        midClimb.set(ControlMode.PercentOutput, -0.4);
        else if(!magSensor.get()){
        midClimb.set(ControlMode.PercentOutput, 0);
        midClimb.setSelectedSensorPosition(0);
        }
    }

    public synchronized void toggleClimbSolenoid(){
        if(midQuickRelease.get() != Value.kForward)
        midQuickRelease.set(Value.kForward);
      else if(midQuickRelease.get() != Value.kReverse)
        midQuickRelease.set(Value.kReverse);
    }

    public synchronized void toggleHighSolenoid(){
        if(intakeSolenoid.get() != Value.kForward)
          intakeSolenoid.set(Value.kForward);
        else if(intakeSolenoid.get() != Value.kReverse)
          intakeSolenoid.set(Value.kReverse);
      }


    @Override
    public void logData() {
        SmartDashboard.putNumber("mid height", midClimb.getSelectedSensorPosition());
    

  


        SmartDashboard.putString("mid solenoid", midQuickRelease.get().toString());

        SmartDashboard.putBoolean("magnet detected?", magSensor.get());
        SmartDashboard.putString("midclimb", midQuickRelease.get().toString());
        SmartDashboard.putNumber("mid climb encoder", midClimb.getSelectedSensorPosition());
        
    }

    @Override
    public void selfTest() {
        midClimb.configFactoryDefault();

        
    }

    
}