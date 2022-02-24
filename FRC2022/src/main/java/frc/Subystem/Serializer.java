package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.util.LidarLite;
import frc.util.Shooting.ShotGenerator;
import frc.util.Shooting.ShotGenerator.ShooterSpeed;
import frc.util.Threading.Threaded;

public class Serializer extends Threaded {

    TalonSRX intakeMotor;
    CANSparkMax elevator;

    IntakeState intakeState;

    Shooter shooter = new Shooter();
    Climb climber = new Climb();
    LED led = new LED();

    private DigitalInput beamBreak;

    ColorSensorV3 colorSensor;
    ColorMatch matches = new ColorMatch();

    ShotGenerator shotGen;

    boolean overrideTarmacShot;
    boolean atSpeed;

    DoubleSolenoid intakeSolenoid;

    private static Serializer instance;

    public static Serializer getInstance(){
        if(instance==null)
            instance = new Serializer();
        return instance;
    }


    public Serializer(){
        matches.addColorMatch(Color.kBlue);
        matches.addColorMatch(Color.kRed);
        matches.addColorMatch(Color.kBlack);
        matches.addColorMatch(Color.kGray);

        intakeMotor = new TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
        intakeMotor.setInverted(true);
        elevator = new CANSparkMax(IntakeConstants.DEVICE_ID_INDEXER,MotorType.kBrushless);
        intakeMotor.setInverted(true);
        elevator.setInverted(true);

        beamBreak = new DigitalInput(0);

        //intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
        /*
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.enableCurrentLimit(true);

        TalonSRXConfiguration intakeConfig = new TalonSRXConfiguration();
        intakeConfig.openloopRamp = IntakeConstants.OPEN_LOOP_RAMP;
        intakeConfig.voltageCompSaturation = 8;
        intakeConfig.peakCurrentLimit = 0;
        intakeConfig.peakOutputForward = .7;
        intakeConfig.peakOutputReverse = -.7;
        
        */
    }

    private enum IntakeState{
        OFF,
        CHEW,
        SWALLOW,
        SLURP,
        SPIT,
        VOMIT,
        EXTEND
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
                index();
                break;
            case SLURP:
                runBoth();
                break;
            case SPIT:
                //shooter.atSpeed(1500, 1000);
                shoot(2000,3000);
                break;
            case VOMIT:
                SmartDashboard.putString("Intake State", "Ejecting");
                ejectAll();
                break;
            //case EXTEND:
        }
        if(intakeState != IntakeState.SPIT)
            shooter.atSpeed(0, 0);
        SmartDashboard.putBoolean("in", beamBreak.get());
    }

    private void intake(){
        intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.INTAKE_SPEED);
    }

    private void index(){
        //elevator.set(IntakeConstants.INDEX_SPEED);
        SmartDashboard.putNumber("bus voltage", elevator.getBusVoltage());
        SmartDashboard.putNumber("output current", elevator.getOutputCurrent());

        elevator.set(-.3);
    }

    private void climb(){
        climber.climb(Robot.operator.getRawAxis(1));
    }

    private void runBoth(){
        intakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_SPEED);
        //if(hasBallStowed()){
        elevator.set(IntakeConstants.INDEX_SPEED);
       // }
    }

    public synchronized void runAll(){
        //intakeMotor.set( IntakeConstants.INTAKE_SPEED);
        elevator.set(IntakeConstants.INDEX_SPEED);
    }

    private void ejectAll(){
        //intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
        elevator.set( -IntakeConstants.INDEX_SPEED);
    }



    private void oneButtonShot(boolean overrideTarmacShot,boolean high){
        ShooterSpeed desiredShot; 
        double arbtirarySpeed;
        if(overrideTarmacShot&&onTarmacLine()){
            if(high){
                arbtirarySpeed = 1500;
            }
            else
                arbtirarySpeed = 1200;
        }


        //if(shooter.atSpeed(arbtirarySpeed))
            index();
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

    private void automatedShot(int RPM){
        //atSpeed = shooter.atSpeed(RPM);
        //shooter.rev(RPM);
        if(RPM == 0)
            //shooter.rev(0);
       // else if(atSpeed)
            index();
        else
        ejectAll();

    }

    private void setArbitrary(){
        //shooter.rev(1700);
    }

    private void updateOff(){
        elevator.set(0);
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
        if(intakeSolenoid.get() == Value.kForward)
            intakeSolenoid.set(Value.kReverse);

        intakeState = IntakeState.SLURP;
    }
    
    public synchronized void setOff(){
        intakeState = IntakeState.OFF;
    }

    public boolean onTarmacLine(){
        
        if(getCurrentColor()==Color.kRed || getCurrentColor()==Color.kBlue)
            return true;
        else
            return false;
    }

    public synchronized void retractIntake(){
        intakeSolenoid.set(Value.kForward);
        SmartDashboard.putBoolean("forward?", intakeSolenoid.get() == Value.kForward);
    }

    public Color getCurrentColor(){
        ColorMatchResult result = matches.matchClosestColor(colorSensor.getColor());
        SmartDashboard.putString("Current Color Sensed", result.color.toString());
        SmartDashboard.putNumber("Confidence", result.confidence);
        return result.color;
    }

    private boolean hasBallStowed(){
        SmartDashboard.putBoolean("stowed ball", !beamBreak.get());
        return beamBreak.get() == IntakeConstants.STOWED;
    }

    public synchronized void adjustShooterSpeeds(double top, double bot){
        shooter.adjustShooterSpeeds(top, bot);
    }
}