// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.trobot5013lib.AverageOverTime;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSub extends SubsystemBase {

  private final QuestNav questNav = new QuestNav();
  private final CommandSwerveDrivetrain estConsumer;
  private final AverageOverTime mLatencyAvg = new AverageOverTime(200);

  private boolean mDisableQuest = false;

  /** Creates a new QuestNav. */
  public QuestNavSub(CommandSwerveDrivetrain estConsumer) {
    this.estConsumer = estConsumer;
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    SmartDashboard.putBoolean(
      "QuestNav/Connected",
      questNav.isConnected()
    );

    SmartDashboard.putBoolean(
      "QuestNav/Tracking",
      questNav.isTracking()
    );

    mLatencyAvg.addMeasurement(questNav.getLatency(), System.currentTimeMillis() );
    SmartDashboard.putNumber(
      "QuestNav/Latency",
      mLatencyAvg.getAverage(System.currentTimeMillis())
    );

    if(!mDisableQuest) {
      processQuestNav();
    }
  }

  private void processQuestNav() {
    if (!questNav.isConnected()) {
      return;
    }

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame poseFrame : poseFrames) {
      if (!poseFrame.isTracking()) {
          continue;
      }

      Pose3d questPose =
          poseFrame.questPose3d();

      Pose3d robotPose =
        questPose.transformBy(
          VisionConstants
              .QuestNavConstants
              .kRobotToQuest
              .inverse()
        );

      double timestamp = poseFrame.dataTimestamp();

      estConsumer.addVisionMeasurement(
          robotPose.toPose2d(),
          timestamp,
          VisionConstants
              .QuestNavConstants
              .kQuestNavStdDevs
      );

      // SmartDashboard.putNumber(
      //     "QuestNav/RobotX",
      //     robotPose.getX()
      // );

      // SmartDashboard.putNumber(
      //     "QuestNav/RobotY",
      //     robotPose.getY()
      // );

      // SmartDashboard.putNumber(
      //       "QuestNav/RobotYaw",
      //     Math.toDegrees(
      //     robotPose.getRotation().getZ()
      //     )
      // );

      // SmartDashboard.putNumber(
      //   "QuestNav/Timestamp",
      //   timestamp
      // );

      // SmartDashboard.putNumber(
      //   "QuestNav/Frame",
      //   poseFrame.frameCount()
      // );
    }
  }

  public void resetQuestNavPose() {
    Pose2d robotPose =
        estConsumer
            .getState()
            .Pose;

    resetQuestNavPose(robotPose);
  }

  public void resetQuestNavPose(Pose2d robotPose) {
    Pose3d questPose =
      new Pose3d(robotPose)
          .transformBy(
              VisionConstants
                  .QuestNavConstants
                  .kRobotToQuest
          );

    questNav.setPose(
        questPose
    );
  }

  public Command resetQuestNavPoseC() {
    return runOnce(
        this::resetQuestNavPose
    );
  }
  
  public void setDisabled(boolean disabled) {
    mDisableQuest = disabled;
  }

  public Command setDisabledC(Pose2d robotPose) {
    return runOnce(
        () -> {
          setDisabled(true);
        }
    );
  }
}
