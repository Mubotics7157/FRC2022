package frc.util;

import frc.robot.Constants;

public class CommonConversions {
  
     public static double stepsToMeters(double steps, double GR){
      return steps*((.1524 * Math.PI) / (2048* GR));


  }


  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(double stepsPerDecisec,double GR) {
    return stepsToMeters(stepsPerDecisec * 10,GR);
  }

    /**
   * Converts from meters to encoder units.
   * @param meters meters
   * @return encoder units
   */
  public static double metersToSteps(double meters, double GR) {
    return (meters / 0.1524 / Math.PI) *2048*GR;
  }

    /**
   * Converts from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec, double GR) {
    return metersToSteps(metersPerSec,GR) * .1d;
  }

  public static double RPMToStepsPerDecisec(double velRPM, double GR){
     return velRPM * 2048/600*1.83333;
  }

  public static double stepsPerDecisecToRPM(double nativeVel,double GR){
    return nativeVel/2048*600/1.83333;
  }

}
