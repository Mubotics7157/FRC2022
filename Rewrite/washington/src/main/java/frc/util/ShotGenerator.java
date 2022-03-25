package frc.util;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotGenerator {

    public class ShooterSpeed {

        public final double topSpeed;
        public final double bottomSpeed; 
        public ShooterSpeed(double topSpeed, double bottomSpeed) { 
          this.topSpeed = topSpeed; 
          this.bottomSpeed = bottomSpeed;
        } 
    }


    
    //log data as {top wheel RPM, distance it made it in}
    private Double[][] topRPM = {
        {2.05d,2.30d,2.5d,2.76,3.25}, // X (distance)
        {1250d,1300d,1550d,2000d,3500d} // Y (ratio of bottom wheel to top wheel)
    };
    private Double[][] ratio = {
        {2.05d,2.30d,2.5,2.76,3.25}, // X (distance)
        {1.2d,1.08d,.9,.65,.3} // Y (RPM)
    };
    SplineInterpolator normalInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(topRPM[0]), Arrays.asList(topRPM[1]));
    SplineInterpolator ratioInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(ratio[0]), Arrays.asList(ratio[1]));

    public ShooterSpeed getShot(double distance) {
        SplineInterpolator speedInterpolator = normalInterpolator;
        SplineInterpolator spinInterpolator = ratioInterpolator;

        double rpm = speedInterpolator.interpolate(distance);
        double shotRatio = spinInterpolator.interpolate(distance);
        SmartDashboard.putNumber("interpolated top", rpm);
        SmartDashboard.putNumber("interpolated ratio", shotRatio);
        return new ShooterSpeed(rpm, rpm*shotRatio);
    }
}