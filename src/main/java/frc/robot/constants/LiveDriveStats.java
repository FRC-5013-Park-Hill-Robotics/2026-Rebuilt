// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;

/* See TroDocs for explanation on why this exists. */
public final class LiveDriveStats {
    public static double OUTPUT_X = 0;
    public static double OUTPUT_Y = 0;
    public static double OUTPUT_H = 0;

    public static Pose2d CURRENT_SHOOT_TARGET1 = new Pose2d();
    public static Pose2d CURRENT_SHOOT_TARGET2 = new Pose2d();

    public static Pose2d DYNAMIC_SHOOT_TARGET = new Pose2d();

    public static boolean AUTO_SHOOTING = true; //Enables/Disables some commands from setting shooter speed
}
