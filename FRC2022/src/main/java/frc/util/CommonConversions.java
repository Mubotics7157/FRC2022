package frc.util;

import frc.robot.Constants;

public class CommonConversions {
  
     public static double stepsToMeters(double steps){
      return steps*((.1524 * Math.PI) / (2048* Constants.DriveConstants.GEAR_RATIO));


  }


  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(double stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

    /**
   * Converts from meters to encoder units.
   * @param meters meters
   * @return encoder units
   */
  public static double metersToSteps(double meters) {
    return (meters / 0.1524 / Math.PI) *2048*Constants.DriveConstants.GEAR_RATIO;
  }

    /**
   * Converts from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

  public static double RPMToStepsPerDecisec(double velRPM){
     return velRPM * 2048/600*1.83333;
  }

  public static double stepsPerDecisecToRPM(double nativeVel){
    return nativeVel/2048*600/1.83333;
  }

}
