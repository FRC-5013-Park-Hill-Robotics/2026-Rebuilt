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

  public final static double IDLE_SPEED_BACK = 50;
  public final static double OUTTAKE_SPEED_BOTTOM = 0;

  public final static double SHOOTER_HEIGHT_FROM_FLOOR = 0;

  public final static class RollerGains {
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kF = 0;
    public static final double kS = 0.39545;
    public static final double kV = 0.12212;
    public static final double kA = 0.0046099;
  }
  public static final class TargetConstants{
    //distance covered, time, and shooter speed in rps
    public static final double[][] TESTS_ARRAY = {
      {0,0,0},
      {1,5,0},
      {2,10,0},
      {3,15,0}
    };
    //Initial velocity and angle for each shooterspeed
    public static final double[][] SHOOTER_DATA = {
      {2000, 2.0, 0.4, 10.0},
      {2500, 4.0, 0.6, 12.0},
      {3000, 6.0, 0.8, 15.0},
      {3500, 8.0, 1.1, 18.0}
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