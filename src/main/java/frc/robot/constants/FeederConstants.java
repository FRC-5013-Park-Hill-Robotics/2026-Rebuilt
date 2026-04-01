// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.trobot5013lib.LinearInterpolator;

/** Add your docs here. */
public final class FeederConstants {
  public final static double OUTTAKE_SPEED = 100;
  
  public final static class RollerGains {
    public static final double kP = 0.3;
    public static final double kI = 0;
    public static final double kD = 0.04;
    public static final double kF = 0;
    public static final double kS = 0.39545;
    public static final double kV = 0.12212;
    public static final double kA = 0.0046099;
  }
}