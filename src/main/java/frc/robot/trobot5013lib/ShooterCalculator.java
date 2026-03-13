// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib;

/** Add your docs here. */
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterCalculator {

    public static class ShotData {
        public double rollerSpeed;
        public double timeOfFlight;
        public double exitVelocity;
        public Rotation2d headingError;

        public ShotData(double rollerSpeed, double timeOfFlight, double exitVelocity, Rotation2d headingError) {
            this.rollerSpeed = rollerSpeed;
            this.timeOfFlight = timeOfFlight;
            this.exitVelocity = exitVelocity;
            this.headingError = headingError;
        }
    }

    /**
     * Calculates the required shot parameters considering robot velocity.
     * @param vx Robot velocity in X (m/s)
     * @param vy Robot velocity in Y (m/s)
     * @param robotPose Current Odometry Pose
     * @param targetPose Goal Position (Field Relative)
     * @param SHOOTER_DATA An array structured like the one bellow
     *     Format: {RollerSpeed, Distance, TOF, ExitVelocity}
     *     public static final double[][] SHOOTER_DATA = {
     *          {2000, 2.0, 0.4, 10.0},
     *          {2500, 4.0, 0.6, 12.0},
     *          {3000, 6.0, 0.8, 15.0},
     *          {3500, 8.0, 1.1, 18.0}
     *      };
     */
    public static ShotData calculateShot(double vx, double vy, Pose2d robotPose, Pose2d targetPose, double[][] shooterData) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double rawDistance = Math.hypot(dx, dy);

        // --- Iterative Solver ---
        // We start with the TOF for the static distance, then adjust.
        double bestTOF = interpolate(rawDistance, 1, 2, shooterData); 
        double effectiveDistance = rawDistance;

        // Two iterations is usually plenty for FRC scales
        for (int i = 0; i < 2; i++) {
            double virtualX = dx - (vx * bestTOF);
            double virtualY = dy - (vy * bestTOF);
            effectiveDistance = Math.hypot(virtualX, virtualY);
            bestTOF = interpolate(effectiveDistance, 1, 2, shooterData);
        }

        // --- Final Calculations ---
        double virtualX = dx - (vx * bestTOF);
        double virtualY = dy - (vy * bestTOF);
        
        double speed = interpolate(effectiveDistance, 1, 0, shooterData);
        double exitVel = interpolate(effectiveDistance, 1, 3, shooterData);
        
        // Calculate field-relative angle to the "virtual" target
        Rotation2d targetAngle = new Rotation2d(Math.atan2(virtualY, virtualX));
        
        // Heading error = Target angle - Robot angle
        Rotation2d error = targetAngle.minus(robotPose.getRotation());

        return new ShotData(speed, bestTOF, exitVel, error);
    }

    private static double interpolate(double distance, int inputCol, int outputCol, double[][] shooterData) {
        if (distance <= shooterData[0][1]) return shooterData[0][outputCol];
        if (distance >= shooterData[shooterData.length - 1][1]) 
            return shooterData[shooterData.length - 1][outputCol];

        for (int i = 0; i < shooterData.length - 1; i++) {
            if (distance >= shooterData[i][1] && distance <= shooterData[i + 1][1]) {
                double x0 = shooterData[i][1];
                double y0 = shooterData[i][outputCol];
                double x1 = shooterData[i + 1][1];
                double y1 = shooterData[i + 1][outputCol];
                return y0 + (distance - x0) * (y1 - y0) / (x1 - x0);
            }
        }
        return shooterData[0][outputCol];
    }
}