package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.fasterxml.jackson.databind.DeserializationConfig;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.util.LidarLite;
import frc.util.Shooting.ShotGenerator;
import frc.util.Shooting.ShotGenerator.ShooterSpeed;
import frc.util.Threading.Threaded;

public class Intake extends Threaded {

    TalonSRX intakeMotor;
    TalonSRX indexerMotor;

    IntakeState intakeState;

    Shooter shooter = new Shooter();
    Climb climber = new Climb();

    private DigitalInput beamBreak;

    ColorSensorV3 colorSensor;
    ColorMatch matches = new ColorMatch();

    ShotGenerator shotGen;

    DoubleSolenoid intakeSolenoid;

    boolean overrideTarmacShot;

    LidarLite lidar = new LidarLite(new DigitalInput(0));

    public Intake(){
        matches.addColorMatch(Color.kBlue);
        matches.addColorMatch(Color.kRed);
        matches.addColorMatch(Color.kBlack);
        matches.addColorMatch(Color.kGray);

        intakeMotor = new TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.enableCurrentLimit(true);

        TalonSRXConfiguration intakeConfig = new TalonSRXConfiguration();
        intakeConfig.openloopRamp = IntakeConstants.OPEN_LOOP_RAMP;
        intakeConfig.voltageCompSaturation = 8;
        intakeConfig.peakCurrentLimit = 0;
        intakeConfig.peakOutputForward = .7;
        intakeConfig.peakOutputReverse = -.7;
        intakeMotor.configAllSettings(intakeConfig);
    }

    private enum IntakeState{
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
                oneButtonShot(true,true);
                break;
            case VOMIT:
                ejectAll();
                break;
            case EXTEND:

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

    private void climb(){
        climber.climb(Robot.operator.getRawAxis(1));
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

        else{
            desiredShot = shotGen.getShot(lidar.getDistance(), high); 
            arbtirarySpeed = desiredShot.speedRPM;
        }

        if(shooter.atSpeed(arbtirarySpeed))
            index();
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
    

    public boolean onTarmacLine(){
        if(getCurrentColor()==Color.kRed || getCurrentColor()==Color.kBlue)
            return true;
        else
            return false;
    }

    public Color getCurrentColor(){
        ColorMatchResult result = matches.matchClosestColor(colorSensor.getColor());
        SmartDashboard.putString("Current Color Sensed", result.color.toString());
        SmartDashboard.putNumber("Confidence", result.confidence);
        return result.color;
    }
}