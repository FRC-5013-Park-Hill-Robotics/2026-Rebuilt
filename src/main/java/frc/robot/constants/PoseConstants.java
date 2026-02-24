package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;

public class PoseConstants {
    public static final Pose2d BLUE_HUB = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_HUB = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));

    public static final Pose2d BLUE_LEFT = new Pose2d(0, -1, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_RIGHT = new Pose2d(0, 1, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_LEFT = new Pose2d(0, -1, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_RIGHT = new Pose2d(0, 1 , Rotation2d.fromDegrees(0));

    public static final List<Pose2d> BLUE_SHUTTLE_POSES = new ArrayList<Pose2d>(Arrays.asList(BLUE_LEFT, BLUE_RIGHT));
    public static final List<Pose2d> RED_SHUTTLE_POSES = new ArrayList<Pose2d>(Arrays.asList(RED_LEFT, RED_RIGHT));
}
