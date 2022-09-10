package frc.util.sensors;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

public class PhotoElectric {
    public AnalogInput photoElectric;
    private double lastState;
    private double lastOpen;
    Timer timer = new Timer();

    public enum PhotoElectricState{
        BROKEN,
        OPEN
    }
    public PhotoElectric(int channel){
        photoElectric = new AnalogInput(channel);
    }

    public PhotoElectricState check(){
        if(photoElectric.getAverageValue()<5){
            if (Math.abs(photoElectric.getAverageValue()-lastState) > 10){
                timer.reset();
                timer.start();
            }
            lastState = photoElectric.getAverageValue();
            return PhotoElectricState.BROKEN;
        }

        else{
            if (Math.abs(photoElectric.getAverageValue())-lastState > 10)
                timer.stop();
            lastState = photoElectric.getAverageValue();
            return PhotoElectricState.OPEN;

        }
    }
    
    public double getLastOpen(){
        return timer.get();
    }

    public boolean isBroken(){
        //if value is 4000 give or take (not broken) return false
        //else if the value is like 4 (broken) give or take return true
        if(photoElectric.getValue() > 1000)
        return false;
        else
        return true;
    }

    public boolean chagedState(){

        return Math.abs(photoElectric.getAverageValue())-lastState > 10;
    }
}
