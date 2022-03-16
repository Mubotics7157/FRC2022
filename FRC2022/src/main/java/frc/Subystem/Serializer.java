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
import frc.util.Threading.Threaded;

public class Serializer extends Threaded {

    TalonSRX intakeMotor;
    CANSparkMax feeder;

    IntakeState intakeState;

    Shooter shooter = new Shooter();

    private DigitalInput beamBreak;

    double topSpeed = 1250;
    double bottomSpeed = 1250/1.08;


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

        beamBreak = new DigitalInput(1);

        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    }

    private enum IntakeState{
        OFF,
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
            case OFF:
                SmartDashboard.putString("Intake State", "Off");
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
                shoot(topSpeed,bottomSpeed);
                break;
            case VOMIT:
                SmartDashboard.putString("Intake State", "Ejecting");
                ejectAll();
                break;
        }
    }

    private void intake(){
        intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.INTAKE_SPEED);
    }

    private void index(){
        feeder.set(IntakeConstants.INDEX_SPEED);

    }

    private void runBoth(){
        intakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_SPEED);
        //if(hasBallStowed())
        feeder.set(IntakeConstants.INDEX_SPEED);
        shoot(-530, -530);
    }

    private void ejectAll(){
        intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.INTAKE_SPEED);
    }

    private void shoot(double top, double bot){
        if(shooter.atSpeed(top, bot)){
            index();
        }
    }

    public synchronized void setArbitrary(double top, double bot){
        if(shooter.atSpeed(top, bot))
            index();
    }

    private void stopMotors(){
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
        stopMotors();
    }


    public synchronized void toggleIntake(boolean down){
        if(!down)
            stopMotors();
        intakeSolenoid.set(down? IntakeConstants.INTAKE_DOWN:IntakeConstants.INTAKE_UP);
    }

    private boolean hasBallStowed(){
        return beamBreak.get() != IntakeConstants.STOWED;
    }

    private synchronized void spitBall(){
        feeder.set(-IntakeConstants.INDEX_SPEED);
    }

    public synchronized void setShooterSpeed(double topSpeed, double botSpeed){
        this.topSpeed = topSpeed;
        this.bottomSpeed = botSpeed;
    }
    
}