// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.LiveDriveStats;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LauncherRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DynamicTargetPose extends Command {

  private CommandXboxController m_gamepad;

  /** Creates a new Dynamic_Target_Pose. */
  public DynamicTargetPose(CommandXboxController mOpperatorController) {
    m_gamepad = mOpperatorController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationX = m_gamepad.getLeftX()*0.05;
		double translationY = m_gamepad.getLeftY()*0.05;

    Pose2d alias = LiveDriveStats.DYNAMIC_SHOOT_TARGET;
    LiveDriveStats.DYNAMIC_SHOOT_TARGET = new Pose2d(alias.getX()+translationX, alias.getY()+translationY, new Rotation2d(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
