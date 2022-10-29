package frc.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.util.AbstractSubsystem;
import frc.util.CommonConversions;
import frc.util.ShotGenerator;
import frc.util.ShotGenerator.ShooterSpeed;

public class Shooter extends AbstractSubsystem {
    
    private TalonFX flywheelBot;
    private TalonFX flywheelTop;

    private static Shooter instance = new Shooter();


    private boolean interpolate = true;
    private ShotGenerator shotGen = new ShotGenerator();
    private ShooterSpeed shooterSpeeds = shotGen.generateArbitraryShot(1350, 1350*1.08);
    private double shotAdj = 1;

    public static Shooter getInstance(){
        return instance;
    }

    public Shooter(){
        super(20);

        flywheelBot = new TalonFX(ShooterConstants.DEVICE_ID_BOT_WHEEL);
        flywheelTop = new TalonFX(ShooterConstants.DEVICE_ID_TOP_WHEEL);

        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        topConfig.slot0.kP = .35;
        topConfig.slot0.kD = .045;
        topConfig.slot0.kF = .05;
        flywheelTop.configAllSettings(topConfig);
        TalonFXConfiguration botConfig = new TalonFXConfiguration();

        botConfig.slot0.kP =.35;    
        botConfig.slot0.kD = .048;
        botConfig.slot0.kF =.05;
        flywheelBot.configAllSettings(botConfig);

        flywheelBot.setNeutralMode(NeutralMode.Coast);
        flywheelTop.setNeutralMode(NeutralMode.Coast);

        flywheelBot.overrideLimitSwitchesEnable(false);
        flywheelTop.overrideLimitSwitchesEnable(false);

        flywheelBot.enableVoltageCompensation(true);
        flywheelTop.enableVoltageCompensation(true);

        flywheelTop.configVoltageCompSaturation(10);
        flywheelBot.configVoltageCompSaturation(10);

        flywheelTop.configVelocityMeasurementWindow(1, 1);
        flywheelBot.configVelocityMeasurementWindow(1, 1);
    }

    public enum ShooterMode{
        SHOOT,
        SPIT,
        DEBUG,
        STATIC_SHOT
    }

    @Override
    public void update() {
        if(interpolate){
            shooterSpeeds = shotGen.getShot(VisionManager.getInstance().getDistanceToTarget());
        }
        rev();
        
        if(Drive.getInstance().isAligned()&&atSpeed()){
            Robot.driver.setRumble(RumbleType.kLeftRumble, .5);
            Robot.driver.setRumble(RumbleType.kRightRumble, .5);
        }
    }

    public boolean atSpeed(){
        boolean atSpeed =  (Math.abs(shooterSpeeds.topSpeed * shotAdj - getTopRPM()) < ShooterConstants.TOLERANCE_RPM) && (Math.abs(shooterSpeeds.bottomSpeed * shotAdj-getBotRPM())<ShooterConstants.TOLERANCE_RPM);
        return atSpeed;
    }
    
    public void rev(){
        shotAdj = SmartDashboard.getNumber("shot adjustment", 1);
        flywheelBot.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(shooterSpeeds.bottomSpeed)*shotAdj);
        flywheelTop.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(shooterSpeeds.topSpeed)*shotAdj);
    }

    public synchronized void setSpitting(){
        interpolate = false;
        shooterSpeeds = shotGen.generateArbitraryShot(1500, 300);
    }

    public synchronized void setInterpolating(){
        interpolate = true;
    }

    public synchronized void setStatic(){
        interpolate = false;
        shooterSpeeds = shotGen.generateArbitraryShot(1350, 1390);
    }

    public void editPorportionalGains(double top, double bot){
        flywheelTop.config_kP(0, top);
        flywheelBot.config_kP(0, bot);
    }

    private double getBotRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelBot.getSelectedSensorVelocity()) ;
    }
    private double getTopRPM(){
        return CommonConversions.stepsPerDecisecToRPM(flywheelTop.getSelectedSensorVelocity());
    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("actual top", getTopRPM());
        SmartDashboard.putNumber("actual bot", getBotRPM());
        
        SmartDashboard.putBoolean("shooter at speed", atSpeed());
        
        SmartDashboard.putNumber("top setpoint", shooterSpeeds.topSpeed);
        SmartDashboard.putNumber("bot setpoint", shooterSpeeds.bottomSpeed);
        
        SmartDashboard.putBoolean("interpolating", interpolate);
    }

    @Override
    public void selfTest() {
        
    }

    public synchronized void setShooterMode(ShooterMode mode){
        interpolate = false;
        SmartDashboard.putString("shooter mode", mode.toString());
        switch(mode){
            case SHOOT:
                rev();
                interpolate = true;
            case SPIT:
                shooterSpeeds = shotGen.generateArbitraryShot(1500, 300);
            case STATIC_SHOT:
                shooterSpeeds = shotGen.generateArbitraryShot(SmartDashboard.getNumber("top wheel setpoint", 1500), SmartDashboard.getNumber("top wheel setpoint", 1500)*1.08);
            case DEBUG:
                shooterSpeeds = shotGen.generateArbitraryShot(SmartDashboard.getNumber("top wheel setpoint", 1500), SmartDashboard.getNumber("top wheel setpoint", 1500)*SmartDashboard.getNumber("shooter ratio", 1));
                editPorportionalGains(SmartDashboard.getNumber("flywheel kP",.01),SmartDashboard.getNumber("flywheel kP",.01));

        }
    }


}
