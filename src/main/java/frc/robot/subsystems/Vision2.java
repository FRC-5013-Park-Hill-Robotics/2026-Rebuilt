// package frc.robot.subsystems;

// import frc.robot.Robot;
// import frc.robot.constants.VisionConstants;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import gg.questnav.questnav.PoseFrame;
// import gg.questnav.questnav.QuestNav;


// /**
//  * Vision subsystem combining PhotonVision AprilTag localization
//  * with QuestNav VIO localization.
//  *
//  * PhotonVision:
//  *   - Shooter/front camera
//  *   - Front-left camera
//  *   - Front-right camera
//  *
//  * QuestNav:
//  *   - Provides high-rate field-relative pose measurements
//  *   - Measurements are timestamped
//  *   - Measurements are transformed from Quest pose to robot pose
//  *
//  * Both localization systems feed the same drivetrain pose estimator.
//  */
// public class Vision2 extends SubsystemBase {

//     // PhotonVision Cameras

//     private final PhotonCamera frontLeftCamera;
//     private final PhotonCamera frontRightCamera;

//     // PhotonVision Pose Estimators

//     private final PhotonPoseEstimator frontLeftPhotonEstimator;
//     private final PhotonPoseEstimator frontRightPhotonEstimator;

//     // Drivetrain Pose Consumer


//     private final CommandSwerveDrivetrain estConsumer;

//     // PhotonVision Standard Deviations

//     private Matrix<N3, N1> currentPhotonStdDevs;

//     //QuestNav

//     private final QuestNav questNav;

//    //Simulation

//     private PhotonCameraSim frontLeftCameraSim;
//     private PhotonCameraSim frontRightCameraSim;
//     private PhotonCameraSim shooterCameraSim;

//     private VisionSystemSim visionSim;

//     // Vision State
  
//     private boolean visionUpdatesEnabled = true;


//     /**
//      * Creates the Vision2 subsystem.
//      *
//      * @param estConsumer drivetrain that receives vision measurements
//      */
//     public Vision2(CommandSwerveDrivetrain estConsumer) {

//         this.estConsumer = estConsumer;

//         // PhotonVision Initialization

//         frontLeftCamera =
//             new PhotonCamera(
//                 VisionConstants.kLeftPhoton
//             );

//         frontRightCamera =
//             new PhotonCamera(
//                 VisionConstants.kRightPhoton
//             );

//         frontLeftPhotonEstimator =
//             new PhotonPoseEstimator(
//                 VisionConstants.kTagLayout,
//                 VisionConstants.kLeftCamOffset
//             );

//         frontRightPhotonEstimator =
//             new PhotonPoseEstimator(
//                 VisionConstants.kTagLayout,
//                 VisionConstants.kRightCamOffset
//             );

//         // QuestNav Initialization

//         questNav = new QuestNav();

//         questNav.onConnected(
//             () -> DriverStation.reportWarning(
//                 "QuestNav connected.",
//                 false
//             )
//         );

//         questNav.onDisconnected(
//             () -> DriverStation.reportWarning(
//                 "QuestNav disconnected.",
//                 false
//             )
//         );

//         questNav.onTrackingAcquired(
//             () -> DriverStation.reportWarning(
//                 "QuestNav tracking acquired.",
//                 false
//             )
//         );

//         questNav.onTrackingLost(
//             () -> DriverStation.reportWarning(
//                 "QuestNav tracking lost.",
//                 false
//             )
//         );

//         // Simulation

//         if (Robot.isSimulation()) {

//             visionSim =
//                 new VisionSystemSim("main");

//             visionSim.addAprilTags(
//                 VisionConstants.kTagLayout
//             );


//             SimCameraProperties cameraProperties =
//                 new SimCameraProperties();

//             cameraProperties.setCalibration(
//                 960,
//                 720,
//                 Rotation2d.fromDegrees(90)
//             );

//             cameraProperties.setCalibError(
//                 0.35,
//                 0.10
//             );

//             cameraProperties.setFPS(15);

//             cameraProperties.setAvgLatencyMs(50);

//             cameraProperties.setLatencyStdDevMs(15);

//             frontLeftCameraSim =
//                 new PhotonCameraSim(
//                     frontLeftCamera,
//                     cameraProperties
//                 );

//             frontRightCameraSim =
//                 new PhotonCameraSim(
//                     frontRightCamera,
//                     cameraProperties
//                 );


//             visionSim.addCamera(
//                 shooterCameraSim,
//                 VisionConstants.kFrontCamOffset
//             );

//             visionSim.addCamera(
//                 frontLeftCameraSim,
//                 VisionConstants.kLeftCamOffset
//             );

//             visionSim.addCamera(
//                 frontRightCameraSim,
//                 VisionConstants.kRightCamOffset
//             );
//         }
//     }

//     // Periodic

//     @Override
//     public void periodic() {

//         /*
//          * QuestNav must process its NetworkTables commands periodically.
//          */
//         questNav.commandPeriodic();


//         if (visionUpdatesEnabled) {

//             // PhotonVision

//             processCamera(
//                 frontLeftCamera,
//                 frontLeftPhotonEstimator
//             );

//             processCamera(
//                 frontRightCamera,
//                 frontRightPhotonEstimator
//             );

//             // QuestNav

//             processQuestNav();
//         }

//         // Dashboard

//         SmartDashboard.putBoolean(
//             "Vision Enabled",
//             visionUpdatesEnabled
//         );

//         SmartDashboard.putBoolean(
//             "QuestNav/Connected",
//             questNav.isConnected()
//         );

//         SmartDashboard.putBoolean(
//             "QuestNav/Tracking",
//             questNav.isTracking()
//         );

//         SmartDashboard.putNumber(
//             "QuestNav/Latency",
//             questNav.getLatency()
//         );
//     }

//     // PhotonVision Processing

//     /**
//      * Processes one PhotonVision camera.
//      *
//      * @param camera PhotonVision camera
//      * @param estimator PhotonPoseEstimator associated with camera
//      */
//     private void processCamera(
//         PhotonCamera camera,
//         PhotonPoseEstimator estimator
//     ) {

//         var result = camera.getLatestResult();

//         if (!result.hasTargets()) {
//             return;
//         }

//         Optional<EstimatedRobotPose> estimatedPose =
//             estimator.update(result);


//         if (estimatedPose.isEmpty()) {
//             return;
//         }

//         updatePhotonEstimationStdDevs(
//             estimatedPose,
//             result.getTargets(),
//             estimator
//         );

//         Pose2d pose =
//             estimatedPose
//                 .get()
//                 .estimatedPose
//                 .toPose2d();


//         double timestamp =
//             estimatedPose
//                 .get()
//                 .timestampSeconds;

//         /*
//          * Send PhotonVision measurement to drivetrain.
//          */
//         estConsumer.addVisionMeasurement(
//             pose,
//             timestamp,
//             currentPhotonStdDevs
//         );
//     }

//     // PhotonVision Standard Deviation Calculation

//     /**
//      * Dynamically calculates PhotonVision confidence.
//      *
//      * Multiple AprilTags produce a stronger estimate than a
//      * single distant AprilTag.
//      */
//     private void updatePhotonEstimationStdDevs(
//         Optional<EstimatedRobotPose> estimatedPose,
//         List<PhotonTrackedTarget> targets,
//         PhotonPoseEstimator estimator
//     ) {

//         if (estimatedPose.isEmpty()) {

//             currentPhotonStdDevs =
//                 VisionConstants.kSingleTagStdDevs;

//             return;
//         }


//         Matrix<N3, N1> estimatedStdDevs =
//             VisionConstants.kSingleTagStdDevs;


//         int numberOfTags = 0;

//         double averageDistance = 0.0;


//         for (PhotonTrackedTarget target : targets) {

//             var tagPose =
//                 estimator
//                     .getFieldTags()
//                     .getTagPose(
//                         target.getFiducialId()
//                     );


//             if (tagPose.isEmpty()) {
//                 continue;
//             }


//             numberOfTags++;


//             averageDistance +=
//                 tagPose
//                     .get()
//                     .toPose2d()
//                     .getTranslation()
//                     .getDistance(
//                         estimatedPose
//                             .get()
//                             .estimatedPose
//                             .toPose2d()
//                             .getTranslation()
//                     );
//         }

//         // No valid tags

//         if (numberOfTags == 0) {

//             currentPhotonStdDevs =
//                 VisionConstants.kSingleTagStdDevs;

//             return;
//         }


//         averageDistance /=
//             numberOfTags;

//         // Multiple tags

//         if (numberOfTags > 1) {

//             estimatedStdDevs =
//                 VisionConstants.kMultiTagStdDevs;
//         }

//         // Single very distant tag

//         if (
//             numberOfTags == 1
//             && averageDistance > 4.0
//         ) {

//             currentPhotonStdDevs =
//                 VecBuilder.fill(
//                     Double.MAX_VALUE,
//                     Double.MAX_VALUE,
//                     Double.MAX_VALUE
//                 );

//             return;
//         }

//         // Distance scaling

//         estimatedStdDevs =
//             estimatedStdDevs.times(
//                 1.0
//                 + (
//                     averageDistance
//                     * averageDistance
//                     / 10.0
//                 )
//             );


//         currentPhotonStdDevs =
//             estimatedStdDevs;
//     }

//     // QuestNav Processing

//     /**
//      * Processes every unread QuestNav pose frame.
//      * QuestNav's pose is the pose of the Quest headset.
//      * The headset pose is transformed into the robot pose using
//      * the ROBOT_TO_QUEST transform defined in VisionConstants.
//      */
//     private void processQuestNav() {

//         if (!questNav.isConnected()) {
//             return;
//         }


//         PoseFrame[] poseFrames =
//             questNav.getAllUnreadPoseFrames();

//         /*
//          * Process every unread frame.
//          *
//          * This prevents QuestNav frames from accumulating and
//          * creating localization latency.
//          */
//         for (PoseFrame poseFrame : poseFrames) {

//             // Ignore frames where tracking is unavailable
//             if (!poseFrame.isTracking()) {
//                 continue;
//             }

//             // Get Quest pose

//             Pose3d questPose =
//                 poseFrame.questPose3d();

//             // Convert Quest pose to robot pose

//             Pose3d robotPose =
//                 questPose.transformBy(
//                     VisionConstants
//                         .QuestNavConstants
//                         .kRobotToQuest
//                         .inverse()
//                 );

//             // Get QuestNav measurement timestamp

//             double timestamp =
//                 poseFrame.dataTimestamp();

//             // Add QuestNav measurement to drivetrain

//             estConsumer.addVisionMeasurement(
//                 robotPose.toPose2d(),
//                 timestamp,
//                 VisionConstants
//                     .QuestNavConstants
//                     .kQuestNavStdDevs
//             );

//             // Dashboard telemetry

//             SmartDashboard.putNumber(
//                 "QuestNav/RobotX",
//                 robotPose.getX()
//             );

//             SmartDashboard.putNumber(
//                 "QuestNav/RobotY",
//                 robotPose.getY()
//             );

//             SmartDashboard.putNumber(
//                  "QuestNav/RobotYaw",
//                 Math.toDegrees(
//                 robotPose.getRotation().getZ()
//                 )
//             );

//             SmartDashboard.putNumber(
//                 "QuestNav/Timestamp",
//                 timestamp
//             );

//             SmartDashboard.putNumber(
//                 "QuestNav/Frame",
//                 poseFrame.frameCount()
//             );
//         }
//     }

//     // QuestNav Pose Reset

//     /**
//      * Resets QuestNav to the current drivetrain pose.
//      *
//      * The drivetrain pose is first converted from the robot
//      * coordinate origin to the QuestNav pose origin.
//      */
//     public void resetQuestNavPose() {

//         Pose2d robotPose =
//             estConsumer
//                 .getState()
//                 .Pose;


//         resetQuestNavPose(robotPose);
//     }


//     /**
//      * Resets QuestNav to a supplied robot pose.
//      *
//      * @param robotPose desired robot pose on the field
//      */
//     public void resetQuestNavPose(
//         Pose2d robotPose
//     ) {

//         Pose3d questPose =
//             new Pose3d(robotPose)
//                 .transformBy(
//                     VisionConstants
//                         .QuestNavConstants
//                         .kRobotToQuest
//                 );


//         questNav.setPose(
//             questPose
//         );
//     }


//     /**
//      * Returns a command that resets QuestNav to the
//      * current drivetrain pose.
//      *
//      * @return QuestNav reset command
//      */
//     public Command resetQuestNavPoseC() {

//         return runOnce(
//             this::resetQuestNavPose
//         );
//     }

//     // Vision Enable / Disable

//     /**
//      * Enables vision pose updates.
//      */
//     public void enableVisionUpdates() {

//         visionUpdatesEnabled = true;
//     }


//     /**
//      * Disables vision pose updates.
//      */
//     public void disableVisionUpdates() {

//         visionUpdatesEnabled = false;
//     }


//     /**
//      * Toggles vision pose updates.
//      */
//     public void toggleVisionUpdates() {

//         visionUpdatesEnabled =
//             !visionUpdatesEnabled;
//     }


//     /**
//      * Returns a command that toggles vision pose updates.
//      *
//      * @return toggle vision command
//      */
//     public Command toggleVisionUpdatesC() {

//         return runOnce(
//             this::toggleVisionUpdates
//         );
//     }


//     /**
//      * Returns whether vision updates are currently enabled.
//      *
//      * @return true if enabled
//      */
//     public boolean areVisionUpdatesEnabled() {

//         return visionUpdatesEnabled;
//     }
// }
