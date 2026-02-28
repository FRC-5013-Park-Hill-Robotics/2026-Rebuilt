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
        //ty is the first number, shooter speed in rps is the second
		public static final double[][] TY_SHOOTER_SPEED_ARRAY = {
				{0,0},
				{1,5},
				{2,10},
				{3,15}
		};

		public static final LinearInterpolator LAUNCHER_TY_SHOOTER_SPEED_INTERPOLATOR = new LinearInterpolator(TY_SHOOTER_SPEED_ARRAY);
        //make a skew linear interpolator based on ty

    }
}