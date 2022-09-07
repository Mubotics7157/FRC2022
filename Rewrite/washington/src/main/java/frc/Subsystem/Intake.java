package frc.Subsystem;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.util.AbstractSubsystem;
import frc.util.OrangeUtility;
import frc.util.sensors.PhotoElectric;

public class Intake extends AbstractSubsystem {
    
    public enum IntakeState{
        OFF,
        INTAKE_REVERSE,
        RUN_ALL,
        INDEX
    }

    IntakeState intakeState = IntakeState.OFF;
    WPI_TalonSRX intake = new WPI_TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
    CANSparkMax indexer = new CANSparkMax(IntakeConstants.DEVICE_ID_INDEXER,MotorType.kBrushless);
    private static Intake instance = new Intake();

    private PhotoElectric photoElectric = new PhotoElectric(0);
    private DigitalInput breakBeam = new DigitalInput(1);

    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,IntakeConstants.INTAKE_SOLENOID_FORWARD,IntakeConstants.INTAKE_SOLENOID_REVERSE);

    private Intake(){
        super(40);
        intake.setInverted(false);
        indexer.setInverted(false);

    }

    public static Intake getInstance(){
        return instance;
    }

    @Override
    public void update() {
        IntakeState snapIntakeState;       
        synchronized(this){
            snapIntakeState = intakeState;
        }

        switch(snapIntakeState){
            case OFF:
                break;
            case INTAKE_REVERSE:
                reverseIntake();
                break;
            case RUN_ALL:
                runBoth();
                break;
            case INDEX:
                index();
                break;
        }
    }

    public synchronized void intake(){
        intake.set(IntakeConstants.INDEX_SPEED);
    }
    private void reverseIntake(){
        intake.set(-IntakeConstants.INDEX_SPEED);
    }
    public synchronized void index(){
        indexer.set(.85);
    }
    public synchronized void reverseIndexer(){
        indexer.set(-IntakeConstants.INDEX_SPEED);
    }

    public synchronized void runBoth(){
        intake();
        photoElectric.check();

        if(!photoElectric.isBroken() || !breakBeam.get())
            indexer.set(0);
        else    
            index();
        }

    public synchronized void intakeAndIndex(){
        intake();
        index();
    }

    public synchronized void stopMotors(){
        intake.set(0);
        indexer.set(0);
    }

     public synchronized void toggleIntake(boolean down){
        if(!down)
            stopMotors();
        intakeSolenoid.set(down? IntakeConstants.INTAKE_DOWN:IntakeConstants.INTAKE_UP);
    }

    public synchronized void toggleIntake(){
        intakeSolenoid.set((intakeSolenoid.get()==Value.kForward)?IntakeConstants.INTAKE_DOWN:IntakeConstants.INTAKE_UP);
    }

    public synchronized IntakeState getIntakeState(){
        return intakeState;
    }

    public synchronized void setIntakeState(IntakeState state){
         if(getIntakeState()!=state)
            stopMotors();
        intakeState = state;
    }   

    public synchronized void setIntakeAndIndexing(){
        intakeState = IntakeState.RUN_ALL;
    }
    public synchronized void setIndexing(){
        intakeState = IntakeState.INDEX;
    }

    public synchronized void setOff(){
        // if(getIntakeState()!=IntakeState.OFF)
            stopMotors();
        intakeState = IntakeState.OFF;
    }

    public boolean indexerCleared(){
        return breakBeam.get();
    }

    @Override
    public void logData() {
       SmartDashboard.putString("Intake State", getIntakeState().toString()); 

       SmartDashboard.putBoolean("photo electric", photoElectric.isBroken());
       SmartDashboard.putBoolean("beam break", breakBeam.get());

    }

    @Override
    public void selfTest() {
        runBoth();
    }
}