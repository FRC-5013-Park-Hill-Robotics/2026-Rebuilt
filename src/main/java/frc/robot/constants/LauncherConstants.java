// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.trobot5013lib.LinearInterpolator;

/** Add your docs here. */
public final class LauncherConstants {
  public final static double INITIAL_SPEED_TOP = 0;
  public final static double INITIAL_SPEED_BOTTOM = 0;
  public final static double INITIAL_SPEED_BACK = 0;

  public final static double OUTTAKE_SPEED_BOTTOM = 80;
  public final static double SHOOTER_HEIGHT_FROM_FLOOR = 0;

  public final static double SOLID_POSITION_TOP = 66;
  public final static double SOLID_POSITION_BACK = 26;

  public final static double REV_TIME = 0.05;
  public final static double REV_OFFSET = 0.2;

  public final static class RollerGains {
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0.001;
    public static final double kF = 0;
    public static final double kS = 0.39545;
    public static final double kV = 0.12212;
    public static final double kA = 0.0046099;
  }
  public static final class TargetConstants{
    
    public static final double distCoefficient = 0; //How velocity effects dist
    public static final double leadCoefficient = 0; //Degrees of lead per m/s
    //distance from hub and shooter speed in rps
    //Front, added 0 to all
    public static final double[][] SHOOTER_HUB_DATA1 = {
      {1.9752, 50},
      {2.3242, 51},
      {2.5399, 54},
      {2.8627, 55},
      {3.1603, 58},
      {3.8657, 59},
      {10, 75}
      // {1.9137,48},
      // {2.0623,50},
      // {2.1633,55},
      // {2.4119,66},
      // {2.5345,66},
      // {2.8522,67},
      // {3.1150,74},
      // {3.7284,80},
    };
    //Back, added 0 to all
    public static final double[][] SHOOTER_HUB_DATA2 = {
      {1.9752, 20},
      {2.3242 ,21},
      {2.5399, 23},
      {2.8627, 24},
      {3.1603, 26}, 
      {3.8657, 27},
      {10, 35}
      // {1.9137,17},
      // {2.0623,20},
      // {2.1633,21},
      // {2.4119,25},
      // {2.5345,26},
      // {2.8522,27},
      // {3.1150,28},
      // {3.7284,28},
    };
    public static final LinearInterpolator shooterHubInterpolator1 = new LinearInterpolator(SHOOTER_HUB_DATA1);
    public static final LinearInterpolator shooterHubInterpolator2 = new LinearInterpolator(SHOOTER_HUB_DATA2);

    //distance from ground and shooter speed in rps
    public static final double[][] SHOOTER_GROUND_DATA1 = {
      {0,0},
      {1,0},
      {2,0},
      {3,0}
    };
    public static final double[][] SHOOTER_GROUND_DATA2 = {
      {0,0},
      {1,0},
      {2,0},
      {3,0}
    };
    public static final LinearInterpolator shooterGroundInterpolator1 = new LinearInterpolator(SHOOTER_HUB_DATA1);
    public static final LinearInterpolator shooterGroundInterpolator2 = new LinearInterpolator(SHOOTER_HUB_DATA2);


    //Format: {RollerSpeed, Distance, TOF,  ExitVelocity}
    public static final double[][] SHOOTER_DATA = {
      {4, 1.2319, 6.19, 30.3522342096},
      {8, 1.54178, 6.86, 33.6375603392},
      {12, 1.84404, 7.2, 35.3048690023},
      {16, 2.342896, 7.57, 37.1194605484},
      {20, 2.4765, 7.75, 38.0021122671},
      {24, 2.39776, 8.18, 40.1102695877},
      {28, 3.048, 8.49, 41.6307772755},
      {32, 3.54838, 8.94, 43.837522376},
      {36, 4.03098, 9.32, 45.701035648},
      {40, 3.99415, 9.6, 47.0737586789},
      //{44, 4.0259, 6.04, 29.623582624},
      {48, 4.5847, 12.15, 59.5765937502},
    };
    public static final double AIR_DENSITY = 1.225; // kg/m^3
    public static final double DRAG_COEFFICIENT = 0.47; // For a ball
    public static final double GRAVITY = 9.81;
    public static final double CROSS_SECTION_AREA_OF_FUEL = 0;
    public static final double MASS_OF_FUEL = 0; // kg

    // Simple container for the result
    // public static class ShotData {
    //   public double rollerSpeed;
    //   public double timeOfFlight;
    //   public double exitVelocity;

    //   public ShotData(double rollerSpeed, double timeOfFlight, double exitVelocity) {
    //     this.rollerSpeed = rollerSpeed;
    //     this.timeOfFlight = timeOfFlight;
    //     this.exitVelocity = exitVelocity;
    //   }
    // }

  //   public static final ShotData getShooterParams(double targetDistance, double targetHeight) {
  //     double bestRollerSpeed = 0;
  //     double bestTOF = 0;
  //     double bestExitV = 0;
  //     double minHeightError = Double.MAX_VALUE;

  //     for (int i = 0; i < VI_ANGLE_SHOOTERSPEED.length - 1; i++) {
  //       double v1 = VI_ANGLE_SHOOTERSPEED[i][0];
  //       double angle1 = Math.toRadians(VI_ANGLE_SHOOTERSPEED[i][1]);
  //       double roller1 = VI_ANGLE_SHOOTERSPEED[i][2];

  //       double v2 = VI_ANGLE_SHOOTERSPEED[i+1][0];
  //       double angle2 = Math.toRadians(VI_ANGLE_SHOOTERSPEED[i+1][1]);
  //       double roller2 = VI_ANGLE_SHOOTERSPEED[i+1][2];

  //       // Test the midpoint of these two presets
  //       double testVi = (v1 + v2) / 2.0;
  //       double testAngle = (angle1 + angle2) / 2.0;
        
  //       // NEW: simulateTrajectory now returns an array [height, time]
  //       double[] results = simulateTrajectory(testVi, testAngle, targetDistance);
  //       double reachedHeight = results[0];
  //       double flightTime = results[1];
        
  //       double error = Math.abs(reachedHeight - targetHeight);
  //       if (error < minHeightError) {
  //         minHeightError = error;
  //         bestRollerSpeed = roller1 + (testVi - v1) * (roller2 - roller1) / (v2 - v1);
  //         bestTOF = flightTime;
  //         bestExitV = testVi;
  //       }
  //     }
  //     return new ShotData(bestRollerSpeed, bestTOF, bestExitV);
  //   }

  //   /** Modified to return {height, time} */
  //   private static double[] simulateTrajectory(double v0, double theta, double targetX) {
  //     double x = 0;
  //     double t = 0;
  //     double y = SHOOTER_HEIGHT_FROM_FLOOR;
  //     double vx = v0 * Math.cos(theta);
  //     double vy = v0 * Math.sin(theta);
  //     double dt = 0.01; 

  //     while (x < targetX) {
  //       double v = Math.sqrt(vx * vx + vy * vy);
  //       double dragForce = 0.5 * AIR_DENSITY * Math.pow(v, 2) * DRAG_COEFFICIENT * CROSS_SECTION_AREA_OF_FUEL;
        
  //       double ax = -(dragForce * (vx / v)) / MASS_OF_FUEL;
  //       double ay = -GRAVITY - (dragForce * (vy / v)) / MASS_OF_FUEL;

  //       vx += ax * dt;
  //       vy += ay * dt;
  //       x += vx * dt;
  //       y += vy * dt;
  //       t += dt;

  //       if (y < 0 || vx <= 0) break;
  //     }
  //     return new double[] { y, t };
  //   }
  // }
  }
}