package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

public class PoseConstants {
    public static final double X_BLUE_SHUTTLE_CUTOFF = 5;
    public static final double X_RED_SHUTTLE_CUTOFF = 11.5;

    public static final Pose2d BLUE_HUB = new Pose2d(4.6, 1.8288, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_HUB = new Pose2d(11.9, 1.8288, Rotation2d.fromDegrees(0));

    //From Driver Perspective
    public static final Pose2d BLUE_LEFT = new Pose2d(3.5, 5.750, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_RIGHT = new Pose2d(3.5, 2.250, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_LEFT = new Pose2d(13, 2.250, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_RIGHT = new Pose2d(13, 5.750, Rotation2d.fromDegrees(0));

    public static final List<Pose2d> BLUE_SHUTTLE_POSES = new ArrayList<Pose2d>(Arrays.asList(BLUE_LEFT, BLUE_RIGHT));
    public static final List<Pose2d> RED_SHUTTLE_POSES = new ArrayList<Pose2d>(Arrays.asList(RED_LEFT, RED_RIGHT));
}
