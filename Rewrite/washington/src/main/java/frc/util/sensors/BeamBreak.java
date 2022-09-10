package frc.util.sensors;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.IntakeConstants;

public class BeamBreak {
    private DigitalInput beamBreak;
    private boolean lastState;
    Timer timer = new Timer();

    public BeamBreak(int channel){
        beamBreak = new DigitalInput(channel);
    }

    public void check(){
        if(beamBreak.get()==IntakeConstants.BEAMBREAK_OPEN){
            if (lastState!=IntakeConstants.BEAMBREAK_OPEN){
                timer.reset();
                timer.start();
            }
            lastState = beamBreak.get();
        }

        else{
            if (lastState!=IntakeConstants.BEAMBREAK_CLOSED){
                timer.reset();
                timer.start();
            }

            lastState = beamBreak.get();
        }
    }
    
    public double getLastOpen(){
        return timer.get();
    }

    public boolean isBroken(){
        return beamBreak.get() == IntakeConstants.BEAMBREAK_CLOSED;
    }

    public boolean chagedState(){

        return (lastState == beamBreak.get());
    }
}
