package frc.Subystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.CommonConversions;

public class Shooter {
    TalonFX shooterMotor;
    double toleranceRPM = 50;
   // FlywheelSim shooterSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, jKgMetersSquared);
   // final LinearSystem<N1,N1,N1> flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), jKgMetersSquared, 1);
    FlywheelSim shooterSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 7);
    //final LinearSystem<N1,N1,N1> flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0, 1);
    //TODO tune kalman filter and lqr virtually
    //final KalmanFilter<N1,N1,N1> flywheelObserver = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(3), VecBuilder.fill(.01), .02);
    //final LinearQuadraticRegulator<N1,N1,N1> LQR = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(8), VecBuilder.fill(12), .02); // 2nd param is state excursion (rad/s) 3rd param is control effor (Volts)
    //final LinearSystemLoop<N1,N1,N1> flywheelLoop = new LinearSystemLoop<>(flywheelPlant, LQR, flywheelObserver, 12, .02);
    TalonFXSimCollection shooterSimCollection;



    public Shooter(){
        shooterMotor = new TalonFX(Constants.ShooterConstants.DEVICE_ID_SHOOTER);
        shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        if(Robot.isSimulation())
            shooterSimCollection = shooterMotor.getSimCollection();
        shooterMotor.config_kP(0, .2);

    }

    public boolean atSpeed(double setpoint){
        if(Math.abs(CommonConversions.stepsPerDecisecToRPM(shooterMotor.getSelectedSensorVelocity())-setpoint) < toleranceRPM)
            return true;
        else 
            return false;
    }

    public void rev(double setpointRPM){
       //flywheelLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(setpointRPM)));
       //flywheelLoop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(CommonConversions.stepsPerDecisecToRPM(shooterMotor.getSelectedSensorVelocity()))));
       //flywheelLoop.predict(.02);
       shooterMotor.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(setpointRPM)); // do I need to divide by 12?
       SmartDashboard.putNumber("velocity setpoint", setpointRPM);
       SmartDashboard.putNumber("actual vel", CommonConversions.stepsPerDecisecToRPM(shooterMotor.getSelectedSensorVelocity()));
       SmartDashboard.putNumber("error", setpointRPM-CommonConversions.stepsPerDecisecToRPM(shooterMotor.getSelectedSensorVelocity()));
       runSimulation();

    }

    private void runSimulation(){
        shooterSim.update(.02);
        shooterSim.setInputVoltage(.25*RobotController.getBatteryVoltage());
        shooterSimCollection.setIntegratedSensorVelocity((int)CommonConversions.RPMToStepsPerDecisec(shooterSim.getAngularVelocityRPM()));
    }
    
}
