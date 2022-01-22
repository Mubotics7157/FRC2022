package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.util.Threading.Threaded;

public class Intake extends Threaded {

    TalonSRX intakeMotor;
    TalonSRX indexerMotor;

    IntakeState intakeState;

    Shooter shooter = new Shooter();

    private DigitalInput beamBreak;

    DoubleSolenoid intakeSolenoid;

    LED led = new LED();

    private enum IntakeState{
        CHEW,
        SWALLOW,
        SLURP,
        SPIT,
        VOMIT
    }
    @Override
    public void update() {
        IntakeState snapIntakeState;
        synchronized (this){
            snapIntakeState = intakeState;
        }

        switch(snapIntakeState){
            case CHEW:
                intake();
                break;
            case SWALLOW:
                index();
                break;
            case SLURP:
                runBoth();
                break;
            case SPIT:
                oneButtonShot();
                break;
            case VOMIT:
                ejectAll();
                break;
        }
    }

    public synchronized void retractIntake(){
        intakeSolenoid.set(Value.kReverse);
    }

    private void intake(){
        intakeMotor.set(ControlMode.PercentOutput, -1);
    }

    private void index(){
        indexerMotor.set(ControlMode.PercentOutput, -1);
    }

    private void runBoth(){
        intakeMotor.set(ControlMode.PercentOutput, -1);
        indexerMotor.set(ControlMode.PercentOutput, -1);
    }

    public synchronized void runAll(){
        intakeMotor.set(ControlMode.PercentOutput, -1);
        indexerMotor.set(ControlMode.PercentOutput, -1);
    }

    private void ejectAll(){
        intakeMotor.set(ControlMode.PercentOutput, 1);
        indexerMotor.set(ControlMode.PercentOutput, 1);
    }

    private void oneButtonShot(){
        double RPM = 1000;
        shooter.rev(RPM);
        //if(shooter.atSpeed(RPM))
          /// indexerMotor.set(ControlMode.PercentOutput, -1);
    }


    public synchronized void setIntaking(){
        intakeState = IntakeState.CHEW;
        intakeSolenoid.set(Value.kForward);
    }

    public synchronized void setShooting(){
        intakeState = IntakeState.SPIT;
    }

    public synchronized void setIndexing(){
        intakeState = IntakeState.SWALLOW;
    }

    public synchronized void setEjecting(){
        intakeState = IntakeState.VOMIT;
    }

    public synchronized void setAll(){
        intakeState = IntakeState.SLURP;
    }
    
    private void updateLED(){
    }
}
