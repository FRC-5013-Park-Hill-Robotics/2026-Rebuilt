// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LiveDriveStats;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToPose extends Command {
  /** Creates a new TurnToPose. */

  private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();
  private final PIDController ControllerH = DriveConstants.ControllerH;

  private CommandSwerveDrivetrain m_drivetrain;
  private Pose2d m_targetPose;

  public TurnToPose(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
    ControllerH.enableContinuousInput(-Math.PI, Math.PI);
    m_drivetrain = drivetrain;
    m_targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ErrorX = m_drivetrain.getState().Pose.getX() - m_targetPose.getX();
    double ErrorY = m_drivetrain.getState().Pose.getY() - m_targetPose.getY();

    double ErrorH = Math.atan2(ErrorY, ErrorX) - m_drivetrain.getState().Pose.getRotation().getRadians();

    SmartDashboard.putNumber("PoseErrorX", ErrorX);
    SmartDashboard.putNumber("PoseErrorY", ErrorY);

    double OutputH = MathUtil.clamp(ControllerH.calculate(ErrorH), -DriveConstants.MaxAngularRate, DriveConstants.MaxAngularRate);

    SmartDashboard.putNumber("PoseOutputH", OutputH);

    LiveDriveStats.OUTPUT_H = OutputH;
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
