package frc.util.Shooting;

import java.util.Arrays;


public class ShotGenerator {
    private enum HEIGHTS{
        LOWER_HUB,
        UPPER_HUB
    }

    public class ShooterSpeed {

        public final double speedRPM;
        public ShooterSpeed(double speedRPM) { 
            this.speedRPM = speedRPM;
        } 
    }


    //log data as {top wheel RPM, distance it made it in}
    private Double[][] upperHub = {
        {0d,0d,0d}, // X (distance)
        {0d,0d,0d} // Y (RPM)
    };
    
    private Double[][] lowerHub = {
        {0d, 0d, 0d, 0d, 0d, 0d}, // X (distance)
        {0d, 0d, 0d, 0d, 0d, 0d} // Y (RPM)
    };
    

    SplineInterpolator lowerInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(lowerHub[1]), Arrays.asList(lowerHub[0]));
    SplineInterpolator upperInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(upperHub[0]), Arrays.asList(upperHub[1]));

    public ShooterSpeed getShot(double distance, boolean upper) {
        SplineInterpolator interpolator;
        HEIGHTS heights;
        if(upper)
            heights = HEIGHTS.UPPER_HUB;
        else
            heights = HEIGHTS.LOWER_HUB;

        switch (heights) {
            case LOWER_HUB:
                interpolator = lowerInterpolator;
                break;
            case UPPER_HUB:
                interpolator = upperInterpolator;
                break;
            default:
                return null;
        }

        double rpm = interpolator.interpolate(distance);
        return new ShooterSpeed(rpm);
    }
}