package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.util.Threading.Threaded;

public class Serializer extends Threaded {

    TalonSRX intakeMotor;
    CANSparkMax feeder;

    IntakeState intakeState;

    Shooter shooter = new Shooter();

    double topSpeed = 1250;
    double bottomSpeed = 1250*1.08;

    DoubleSolenoid intakeSolenoid;

    private static Serializer instance;

    public static Serializer getInstance(){
        if(instance==null)
            instance = new Serializer();
        return instance;
    }


    public Serializer(){
        intakeMotor = new TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
        feeder = new CANSparkMax(IntakeConstants.DEVICE_ID_INDEXER,MotorType.kBrushless);
        feeder.setInverted(false);
        intakeMotor.setInverted(true);


        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    }

    private enum IntakeState{
        OFF,
        REVERSE_INTAKE,
        REVERSE_INDEXER,
        INTAKE_INDEXER,
        SHOOT,
        INTAKE
    }
    @Override
    public void update() {
        IntakeState snapIntakeState;
        synchronized (this){
            snapIntakeState = intakeState;
        }
        switch(snapIntakeState){
            case OFF:
                SmartDashboard.putString("intake state", "OFF");
                break;
            case REVERSE_INTAKE:
                reverseIntake();
                SmartDashboard.putString("intake state", "INTAKE REVERSED");
                break;
            case REVERSE_INDEXER:
                spitBall();
                SmartDashboard.putString("intake state", "INDEXER REVERSED");
                break;
            case INTAKE_INDEXER:
                runBoth();
                SmartDashboard.putString("intake state", "INTAKE + INDEX");
                break;
            case SHOOT:
                shoot(topSpeed,bottomSpeed);
                SmartDashboard.putString("intake state", "SHOOTING");
                break;
            case INTAKE:
                intake();
                SmartDashboard.putString("intake state", "INTAKING");
                break;
        }
        System.out.println("intake updating");
    }

    private void reverseIntake(){
        intakeMotor.set(ControlMode.PercentOutput,-IntakeConstants.INTAKE_SPEED);
    }

    private void index(){
        feeder.set(IntakeConstants.INDEX_SPEED);

    }

    private void intake(){
        intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.INTAKE_SPEED);
    }

    private void runBoth(){
        intakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_SPEED);
        //if(hasBallStowed())
        feeder.set(IntakeConstants.INDEX_SPEED);
        shoot(-130, -130);
    }

    private void shoot(double top, double bot){
        if(DriverStation.isAutonomous()&&shooter.atSpeed(top, bot)){
            index();
            intake();
        }
        else
        shooter.atSpeed(top, bot);
    }

    private void shootAutomated(double top, double bot){
        if(shooter.atSpeed(top, bot))
            index();
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
        if(intakeSolenoid.get()!=Constants.IntakeConstants.INTAKE_DOWN)
            intakeSolenoid.set(Value.kForward);
        intakeState = IntakeState.INTAKE;
    }

    public synchronized void setShooting(){
        intakeState = IntakeState.SHOOT;
    }

    public synchronized void setIndexing(){
        intakeState = IntakeState.REVERSE_INDEXER;
    }

    public synchronized void setIntakeBackwards(){
        intakeState = IntakeState.REVERSE_INTAKE;
    }
    public synchronized void setAll(){
        toggleIntake(true);
        intakeState = IntakeState.INTAKE_INDEXER;
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

    private synchronized void spitBall(){
        feeder.set(-IntakeConstants.INDEX_SPEED);
    }

    public synchronized void setShooterSpeed(double topSpeed, double botSpeed){
        this.topSpeed = topSpeed;
        this.bottomSpeed = botSpeed;
    }

    public synchronized void runIndexer(){
        feeder.set(IntakeConstants.INDEX_SPEED);
    }

    public synchronized void adjustShooterSpeeds(double topAdj, double botAdj){
        topSpeed += topAdj;
        bottomSpeed += botAdj;
    }
    

    
}