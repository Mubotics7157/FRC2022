package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.util.Shooting.ShotGenerator;
import frc.util.Shooting.ShotGenerator.ShooterSpeed;
import frc.util.Threading.Threaded;

public class Serializer extends Threaded {

    TalonSRX intakeMotor;
    CANSparkMax feeder;

    IntakeState intakeState;

    Shooter shooter = new Shooter();
    LED led = new LED();

    private DigitalInput beamBreak;
    Climb climb = new Climb();


    ShotGenerator shotGen;

    DoubleSolenoid intakeSolenoid;

    private static Serializer instance;

    public static Serializer getInstance(){
        if(instance==null)
            instance = new Serializer();
        return instance;
    }


    public Serializer(){
        intakeMotor = new TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
        intakeMotor.setInverted(true);
        feeder = new CANSparkMax(IntakeConstants.DEVICE_ID_INDEXER,MotorType.kBrushless);
        intakeMotor.setInverted(true);
        feeder.setInverted(true);

        beamBreak = new DigitalInput(0);

        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    }

    private enum IntakeState{
        OFF,
        CHEW,
        SWALLOW,
        SLURP,
        SPIT,
        VOMIT,
        EXTEND,
        RETRACT
    }
    @Override
    public void update() {
        IntakeState snapIntakeState;
        synchronized (this){
            snapIntakeState = intakeState;
        }
        switch(snapIntakeState){
            case OFF:
                SmartDashboard.putString("Intake State", "Off");
                updateOff();
                break;
            case CHEW:
                SmartDashboard.putString("Intake State", "Intaking");
                intake();
                break;
            case SWALLOW:
                SmartDashboard.putString("Intake State", "Indexer");
                spitBall();
                break;
            case SLURP:
                runBoth();
                break;
            case SPIT:
                //shooter.atSpeed(1500, 1000);
                shoot(1250,1500);
                break;
            case VOMIT:
                SmartDashboard.putString("Intake State", "Ejecting");
                ejectAll();
                break;
        }
        if(intakeState != IntakeState.SPIT)
            shooter.atSpeed(0, 0);
        if(intakeState !=IntakeState.EXTEND && intakeState!=IntakeState.RETRACT)
        SmartDashboard.putBoolean("in", beamBreak.get());
    }

    private void intake(){
        intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.INTAKE_SPEED);
    }

    private void index(){
        feeder.set(IntakeConstants.INDEX_SPEED);

    }

    private void runBoth(){
        intakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_SPEED);
        feeder.set(IntakeConstants.INDEX_SPEED);
    }

    private void ejectAll(){
        intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.INTAKE_SPEED);
    }

    private void shoot(double top, double bot){
        if(shooter.atSpeed(top, bot)){
            index();
            led.setShooterLED(.77);
            //^^if shooter is at its wanted speed LEDs will be GREEN
        }
        else{
            led.setShooterLED(.69);
            //^^if shooter is not at its wanted speed LEDs will be YELLOW
        }
        
    }

    public synchronized void setArbitrary(double top, double bot){
        if(shooter.atSpeed(top, bot))
            index();
    }

    private void updateOff(){
        feeder.set(0);
        intakeMotor.set(ControlMode.PercentOutput, 0);
        shooter.rev(0, 0);
    }



    public synchronized void setIntaking(){
       // intakeSolenoid.set(Value.kForward);
        intakeState = IntakeState.CHEW;
        //intakeSolenoid.set(Value.kForward);
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
        toggleIntake(true);
        intakeState = IntakeState.SLURP;
    }
    
    public synchronized void setOff(){
        intakeState = IntakeState.OFF;
    }


    public synchronized void toggleIntake(boolean down){
        if(!down)
            setOff();
        intakeSolenoid.set(down? IntakeConstants.INTAKE_DOWN:IntakeConstants.INTAKE_UP);
    }

    private boolean hasBallStowed(){
        SmartDashboard.putBoolean("stowed ball", !beamBreak.get());
        return beamBreak.get() == IntakeConstants.STOWED;
    }

    private synchronized void spitBall(){
        feeder.set(-IntakeConstants.INDEX_SPEED);
    }
    
}