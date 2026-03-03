package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    public static final double moverInTime = 1.5; //Seconds
    public static final double moveInVolt = -10; //Voltage
    public static final double moverOutTime = 1.5; //Seconds
    public static final double moveOutVolt = 5; //Voltage

    public final static class RollerGains {
        public static final double kP = 0.010371;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.39545;
        public static final double kV = 0.12212;
        public static final double kA = 0.0046099;
        //public static final double kIntakeRotation = 36.2942;
    }

    public final static class IntakeMoverGains {
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.39545;
        public static final double kV = 0.12212;
        public static final double kA = 0.0046099;
        //public static final double kIntakeRotation = 36.2942;
    }
}