package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.util.AbstractSubsystem;

public class Intake extends AbstractSubsystem {
    
    public enum IntakeState{
        INTAKE_REVERSE,
        INDEX,
        INTAKE,
        RUN_ALL,
        SHOOTING,
        INDEX_REVERSE
    }

    IntakeState intakeState = IntakeState.RUN_ALL;
    TalonSRX intake = new TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
    CANSparkMax indexer = new CANSparkMax(IntakeConstants.DEVICE_ID_INDEXER,MotorType.kBrushless);

    Shooter shooter = new Shooter();
    
    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,IntakeConstants.INTAKE_SOLENOID_FORWARD,IntakeConstants.INTAKE_SOLENOID_REVERSE);

    double topSpeed = 1350;
    double botSpeed = 1350/1.08;
    double ratio = 1.08;

    private Intake(){
        super(40);
        intake.setInverted(false);
        indexer.setInverted(true);
    }

    @Override
    public void update() {
        IntakeState snapIntakeState;       
        synchronized(this){
            snapIntakeState = intakeState;
        }

        switch(snapIntakeState){
            case INTAKE_REVERSE:
                reverseIntake();
                break;
            case INDEX_REVERSE:
                reverseIndexer();
                break;
            case INTAKE:
                intake();
                break;
            case INDEX:
                break;
            case RUN_ALL:
                runBoth();
                break;
            case SHOOTING:
                break;
        }
    }

    private void intake(){
        intake.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
    }
    private void reverseIntake(){
        intake.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_SPEED);
    }
    private void index(){
        indexer.set(IntakeConstants.INDEX_SPEED);
    }
    private void reverseIndexer(){
        indexer.set(-IntakeConstants.INDEX_SPEED);
    }

    private void runBoth(){
        intake();
        index();
    }

    private void stopMotors(){
        intake.set(ControlMode.PercentOutput,0);
        indexer.set(0);
    }

    public synchronized void setShooterSpeeds(double top, double bot){
        topSpeed = top;
        botSpeed = bot;
    }

    public synchronized void setShooterSpeeds(){
        topSpeed = SmartDashboard.getNumber("top wheel setpoint", 1350);
        botSpeed = SmartDashboard.getNumber("bot wheel setpoint", 1350/1.08);
    }

    public synchronized void setShooterRatio(){
        ratio = SmartDashboard.getNumber("shooter ratio", 1);
    }

    public synchronized IntakeState getIntakeState(){
        return intakeState;
    }

    public synchronized void setIntakeState(IntakeState state){
        intakeState = state;
    }

    @Override
    public void logData() {
       SmartDashboard.putString("Intake State", getIntakeState().toString()); 

       // shooter speed and ratio turning
       SmartDashboard.putNumber("shooter ratio", 1);
       SmartDashboard.getNumber("top wheel setpoint", 1000);
       SmartDashboard.getNumber("bot wheel setpoint", 1000);
    }

    @Override
    public void selfTest() {
        runBoth();
    }
}
