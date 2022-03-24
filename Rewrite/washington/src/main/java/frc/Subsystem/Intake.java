package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.util.AbstractSubsystem;
import frc.util.LidarLite;
import frc.util.OrangeUtility;
import frc.util.ShotGenerator;
import frc.util.Shooting.InterpolatingDouble;
import frc.util.Shooting.InterpolatingTreeMap;
import frc.util.ShotGenerator.ShooterSpeed;

public class Intake extends AbstractSubsystem {
    
    public enum IntakeState{
        OFF,
        INTAKE_REVERSE,
        INDEX,
        INTAKE,
        RUN_ALL,
        SHOOTING,
        INDEX_REVERSE,
        AUTO_SHOT
    }

    IntakeState intakeState = IntakeState.OFF;
    WPI_TalonSRX intake = new WPI_TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
    CANSparkMax indexer = new CANSparkMax(IntakeConstants.DEVICE_ID_INDEXER,MotorType.kBrushless);
    private static Intake instance = new Intake();

    DigitalInput breakBeam = new DigitalInput(1);

    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    ColorMatch colorMatcher = new ColorMatch();

    
    Shooter shooter = new Shooter();
    
    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,IntakeConstants.INTAKE_SOLENOID_FORWARD,IntakeConstants.INTAKE_SOLENOID_REVERSE);

    double topSpeed = 1350;
    double botSpeed = 1350/1.08;
    double ratio = 1.08;

    private boolean useDefault = false;


    Color PassiveColor;

    LidarLite lidar = new LidarLite(new DigitalInput(0));

    ShotGenerator shotGen = new ShotGenerator();


    private Intake(){
        super(40);
        intake.setInverted(false);
        indexer.setInverted(false);


        colorMatcher.addColorMatch(IntakeConstants.BLUE);
        colorMatcher.addColorMatch(IntakeConstants.OTHER_BLUE);
        colorMatcher.addColorMatch(IntakeConstants.OTHER_RED);
        colorMatcher.addColorMatch(IntakeConstants.RED);
        colorMatcher.addColorMatch(IntakeConstants.PASSIVE);
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
            case INDEX_REVERSE:
                reverseIndexer();
                break;
            case INTAKE:
                intake();
                break;
            case INDEX:
                index();
                break;
            case RUN_ALL:
                runBoth();
                break;
            case SHOOTING:
                shoot();
                break;
            case AUTO_SHOT:
                autoShot();
                //mapShot();
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
        indexer.set(.9);
    }
    public synchronized void reverseIndexer(){
        indexer.set(-IntakeConstants.INDEX_SPEED);
    }

    private void autoIntake(){
        if(getSweetSpotStatus())
            indexer.set(-.3);
        else
            stopMotors();
    }

    public synchronized void runBoth(){
        shooter.atSpeed(-250, -250);
        intake();
        index();
    }

    public synchronized void stopMotors(){
        shooter.atSpeed(0, 0);
        intake.set(0);
        indexer.set(0);
    }

    public synchronized void shoot(){
       shooter.atSpeed(topSpeed, botSpeed);
        if(DriverStation.isAutonomous()&&shooter.atSpeed(topSpeed, botSpeed))
            index();
        SmartDashboard.putBoolean("at speed?", shooter.atSpeed(topSpeed, botSpeed));
        if(Robot.driver.getRawAxis(3)>.2)
            indexer.set(IntakeConstants.INDEX_SPEED/2);

    }

    public synchronized void autoShot(){
        if(useDefault){
            shoot();
        }
        else{
            double indexSpeed=.9;
            if(VisionManager.getInstance().getDistanceToTarget()<3)
                indexSpeed = .85;
            ShooterSpeed shooterSpeeds = shotGen.getShot(VisionManager.getInstance().getDistanceToTarget());
            shooter.atSpeed(shooterSpeeds.topSpeed, shooterSpeeds.bottomSpeed);
            if(Robot.driver.getRawAxis(3)>.2)
            indexer.set(IntakeConstants.INDEX_SPEED*indexSpeed);
            SmartDashboard.putNumber("interpolated top", shooterSpeeds.topSpeed);
            SmartDashboard.putNumber("interpolated bot", shooterSpeeds.bottomSpeed);
        }
    }

    public synchronized void mapShot(){
        double wheelSpeed = IntakeConstants.FLYWHEEL_RPM_MAP.getInterpolated(new InterpolatingDouble(VisionManager.getInstance().getDistanceToTarget())).value;
        double wheelRatio = IntakeConstants.FLYWHEEL_RATIO_MAP.getInterpolated(new InterpolatingDouble(VisionManager.getInstance().getDistanceToTarget())).value;

        shooter.atSpeed(wheelSpeed, wheelSpeed*wheelRatio);

        SmartDashboard.putNumber("interpolated top", wheelSpeed);
        SmartDashboard.putNumber("interpolated bot", wheelRatio);

         if(Robot.driver.getRawAxis(3)>.2)
        indexer.set(IntakeConstants.INDEX_SPEED*.8);
    }

     public synchronized void toggleIntake(boolean down){
        if(!down)
            stopMotors();
        intakeSolenoid.set(down? IntakeConstants.INTAKE_DOWN:IntakeConstants.INTAKE_UP);
    }

    public synchronized void toggleIntake(){
        if(intakeSolenoid.get()==Value.kForward)
            intakeSolenoid.set(Value.kReverse);
        else if(intakeSolenoid.get()==Value.kReverse)
            intakeSolenoid.set(Value.kForward);
        OrangeUtility.sleep(500);
    }

    public synchronized void setShooterSpeeds(double top, double bot){
        topSpeed = top;
        botSpeed = bot;
    }


    public synchronized void setShooterSpeeds(){
        topSpeed = SmartDashboard.getNumber("top wheel setpoint", 1350);
        botSpeed = topSpeed*SmartDashboard.getNumber("shooter ratio", 1);
    }

    public synchronized void setShooterRatio(){
        ratio = SmartDashboard.getNumber("shooter ratio", 1);
    }


    public synchronized void calibratePassiveColor(){
        Color sensedColor = colorSensor.getColor();
        PassiveColor = new Color(sensedColor.red,sensedColor.green,sensedColor.blue);
    }

    private boolean getSweetSpotStatus(){
        ColorMatchResult match = colorMatcher.matchClosestColor(colorSensor.getColor());
        SmartDashboard.putBoolean("color?", match.color.equals(IntakeConstants.PASSIVE));
        return match.color.equals(IntakeConstants.PASSIVE);
    }

    public synchronized IntakeState getIntakeState(){
        return intakeState;
    }

    public synchronized void setIntakeState(IntakeState state){
        intakeState = state;
    }

    public synchronized void setIntakeAndIndexing(){
        intakeState = IntakeState.RUN_ALL;
    }

    public synchronized void setOff(){
        stopMotors();
        intakeState = IntakeState.OFF;
    }

    public synchronized void setIndexBackwards(){
        intakeState= IntakeState.INDEX_REVERSE;
    }

    public synchronized void setShooting(){
        intakeState= IntakeState.SHOOTING;
    }

    public synchronized void toggleDefault(){
        useDefault = !useDefault;
    }

    @Override
    public void logData() {
       SmartDashboard.putString("Intake State", getIntakeState().toString()); 

       // shooter speed and ratio turning

       SmartDashboard.putNumber("red component", colorSensor.getColor().red);
       SmartDashboard.putNumber("green component", colorSensor.getColor().green);
       SmartDashboard.putNumber("blue component", colorSensor.getColor().blue);

       SmartDashboard.putBoolean("beam broken?", breakBeam.get());

       SmartDashboard.putNumber("shooter top speed", topSpeed);
       SmartDashboard.putNumber("shooter bot speed", botSpeed);

       SmartDashboard.putNumber("lidar distance", lidar.getDistance());

       SmartDashboard.putBoolean("using default shot?", useDefault);

    }

    @Override
    public void selfTest() {
        runBoth();
    }
}
