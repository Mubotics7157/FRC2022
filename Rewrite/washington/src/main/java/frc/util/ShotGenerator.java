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
        {1.81,2.2,2.41,2.92, 3.03,3.28}, //3.96 last
        {1315d,1445d,1525d,1675d, 1755d,1790d}//#region} //3250d last
    };
    private Double[][] ratio = {
        {1.81,2.2,2.41,2.92, 3.03, 3.28}, //3.96 last
        {1d,.94,.92,.825, 0.8,.725} //.025 last
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

    public ShooterSpeed generateArbitraryShot(double top, double bot){
        return new ShooterSpeed(top, bot);
    }
}