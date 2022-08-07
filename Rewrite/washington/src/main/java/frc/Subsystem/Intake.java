package frc.Subsystem;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.util.AbstractSubsystem;
import frc.util.OrangeUtility;
import frc.util.ShotGenerator;
import frc.util.Shooting.InterpolatingDouble;
import frc.util.ShotGenerator.ShooterSpeed;
import frc.util.sensors.PhotoElectric;

public class Intake extends AbstractSubsystem {
    
    public enum IntakeState{
        OFF,
        INTAKE_REVERSE,
        INDEX,
        INTAKE,
        RUN_ALL,
        SHOOTING,
        INDEX_REVERSE,
        AUTO_SHOT,
        SPIT_BALL,
        PRE_REV
    }

    public enum SensorState{
        //these are saying what beams have been broken
        none,
        indexer,
        intake_indexer,
        indexer_flywheel

    }

    IntakeState intakeState = IntakeState.OFF;
    WPI_TalonSRX intake = new WPI_TalonSRX(IntakeConstants.DEVICE_ID_INTAKE);
    CANSparkMax indexer = new CANSparkMax(IntakeConstants.DEVICE_ID_INDEXER,MotorType.kBrushless);
    private static Intake instance = new Intake();

    private PhotoElectric photoElectric = new PhotoElectric(0);
    private DigitalInput breakBeam = new DigitalInput(1);




    boolean interpolated = false;
    
    Shooter shooter = new Shooter();
    
    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,IntakeConstants.INTAKE_SOLENOID_FORWARD,IntakeConstants.INTAKE_SOLENOID_REVERSE);

    double topSpeed = 1350;
    double botSpeed = 1350*1.08;
    double ratio = 1.08;


    private boolean atSpeed = false;
    double shotAdj = .97;

    private boolean useDefault = false;




    ShotGenerator shotGen = new ShotGenerator();


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

        rampWheels();
        switch(snapIntakeState){
            case OFF:
                //VisionManager.getInstance().toggleLimelightLEDMode(false);
                updateOff();
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
                autoShot();
                index();
                //VisionManager.getInstance().toggleLimelightLEDMode(true);
                break;
            case AUTO_SHOT:
                autoShot();
                break;
            case SPIT_BALL:
                spitBall();
                break;
        }
    }

    public synchronized void updateOff(){
        // if(DriverStation.isTeleop()){
            // if(!breakBeam.get())
                // indexer.set(-.85);
            // else
                // indexer.set(0);
        // }
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
        //shooter.atSpeed(0, 0);
        intake.set(0);
        indexer.set(0);
    }

    public synchronized void holdIntaking(){
        intake.set(0);
        indexer.set(0);
    }
    private void spitBall(){
        if(shooter.atSpeed(1500,300))
            index();
    }

    public synchronized void shoot(){
        if(shooter.atSpeed(topSpeed, botSpeed))
            index();
        else
            atSpeed = shooter.atSpeed(topSpeed, botSpeed);
        

    }

    private void rampWheels(){
        ShooterSpeed shooterSpeeds;
        if(interpolated){
            shooterSpeeds = shotGen.getShot(VisionManager.getInstance().getDistanceToTarget());
        }
        else
            shooterSpeeds = shotGen.generateArbitraryShot(topSpeed, botSpeed);
            
        shooter.atSpeed(shooterSpeeds.topSpeed, shooterSpeeds.bottomSpeed);
    }
    public synchronized void autoShot(){
        ShooterSpeed shooterSpeeds;
            double indexSpeed=.9;
            if(VisionManager.getInstance().getDistanceToTarget()<3)
                indexSpeed = .85;
        if(interpolated){
            shooterSpeeds = shotGen.getShot(VisionManager.getInstance().getDistanceToTarget());
        }
        else
            shooterSpeeds = shotGen.generateArbitraryShot(topSpeed, botSpeed);
            
        if(DriverStation.isAutonomous()&&shooter.atSpeed(shooterSpeeds.topSpeed*shotAdj, shooterSpeeds.bottomSpeed*shotAdj))
            index();
        else
            shooter.atSpeed(shooterSpeeds.topSpeed*shotAdj, shooterSpeeds.bottomSpeed*shotAdj);

        SmartDashboard.putNumber("interpolated top", shooterSpeeds.topSpeed);
        SmartDashboard.putNumber("interpolated bot", shooterSpeeds.bottomSpeed);


    }

    private void preRev(){
        ShooterSpeed shooterSpeeds = shotGen.getShot(VisionManager.getInstance().getDistanceToTarget());
        shooter.atSpeed(shooterSpeeds.topSpeed*shotAdj, shooterSpeeds.bottomSpeed*shotAdj);
    }

    public synchronized void adjustShooterSpeeds(double val){
        shotAdj = 1 + val;
    }


    public synchronized void mapShot(){
        double wheelSpeed = IntakeConstants.FLYWHEEL_RPM_MAP.getInterpolated(new InterpolatingDouble(VisionManager.getInstance().getDistanceToTarget())).value;
        double wheelRatio = IntakeConstants.FLYWHEEL_RATIO_MAP.getInterpolated(new InterpolatingDouble(VisionManager.getInstance().getDistanceToTarget())).value;

        shooter.atSpeed(wheelSpeed, wheelSpeed*wheelRatio);

        SmartDashboard.putNumber("interpolated top", wheelSpeed);
        SmartDashboard.putNumber("interpolated bot", wheelRatio);

    }

     public synchronized void toggleIntake(boolean down){
        if(!down)
            stopMotors();
        intakeSolenoid.set(down? IntakeConstants.INTAKE_DOWN:IntakeConstants.INTAKE_UP);
    }

    private void ocrShot(){
        shooter.atSpeed(1350, 1350*1.08);
        if(Robot.driver.getRawAxis(3)>.2)
            indexer.set(IntakeConstants.INDEX_SPEED*.85);
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
        botSpeed = topSpeed*SmartDashboard.getNumber("shooter ratio", 1.08);
    }

    public synchronized void setShooterRatio(){
        ratio = SmartDashboard.getNumber("shooter ratio", 1);
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

    public synchronized void setOff(){
        // if(getIntakeState()!=IntakeState.OFF)
            stopMotors();
        intakeState = IntakeState.OFF;
    }

    public synchronized void setIndexBackwards(){
        intakeState= IntakeState.INDEX_REVERSE;
    }

    public synchronized void setShooting(){
        intakeState= IntakeState.SHOOTING;
    }

    public synchronized void setPreRevving(){
        intakeState = IntakeState.PRE_REV;
    }

    public synchronized void setSpitting(){
        intakeState = IntakeState.SPIT_BALL;
    }

    public synchronized void toggleDefault(){
        useDefault = !useDefault;
    }

    public synchronized void manualPowerAdjust(){
        shotAdj = SmartDashboard.getNumber("shot adjustment", 1);
    }

    public synchronized void adjustShooterkP(){
        shooter.editPorportionalGains(.3, .3);
    }

    public synchronized void toggleInterpolated(){
        interpolated = !interpolated;
        OrangeUtility.sleep(500);
    }


    public boolean indexerCleared(){
        return breakBeam.get();
    }


    @Override
    public void logData() {
       SmartDashboard.putString("Intake State", getIntakeState().toString()); 

       SmartDashboard.putNumber("shooter top speed", topSpeed);
       SmartDashboard.putNumber("shooter bot speed", botSpeed);



       SmartDashboard.putNumber("proportional adjustment", shotAdj);

       SmartDashboard.putBoolean("interpolated shot", interpolated);
       SmartDashboard.putBoolean("at speed", atSpeed);
       SmartDashboard.putBoolean("ball?", breakBeam.get());
       SmartDashboard.putBoolean("photo electric", photoElectric.isBroken());
       SmartDashboard.putBoolean("beam break", breakBeam.get());

    }

    @Override
    public void selfTest() {
        runBoth();
    }
}