package frc.Subystem;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED {
    Spark shooterLED;
    Spark ballStatusLED;
    Spark looksLED;

    public LED(){
        shooterLED = new Spark(0);
        ballStatusLED = new Spark(1);
        looksLED = new Spark(2);
    }

    public void setShooterLED(double setShooterLED){
        shooterLED.set(setShooterLED);
    }

    public void setBallStatusLED(double setBallLED){
        ballStatusLED.set(setBallLED);
    }

    public void setLooksLED(double setLooksLED){
        looksLED.set(setLooksLED);
    }
}

