// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shuttle extends Command {
  /** Creates a new Shuttle. */

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


  private final PIDController ControllerH = DriveConstants.ControllerH;

  private CommandSwerveDrivetrain m_drivetrain;
  Alliance mAlliance;

  public Shuttle(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    mAlliance = RobotContainer.getAlliance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mAlliance = RobotContainer.getAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get the nearest shuttle pose based on alliance color
    Pose2d m_targetPose = switch (mAlliance) {
      case Red -> m_drivetrain.getState().Pose.nearest(PoseConstants.RED_SHUTTLE_POSES);
      case Blue -> m_drivetrain.getState().Pose.nearest(PoseConstants.RED_SHUTTLE_POSES);
    };

    //Same as TurnToPose, but with the target pose being the nearest shuttle pose instead of a fixed pose
    double ErrorX = m_drivetrain.getState().Pose.getX() - m_targetPose.getX();
    double ErrorY = m_drivetrain.getState().Pose.getY() - m_targetPose.getY();

    double ErrorH = Math.atan2(ErrorY, ErrorX) - m_drivetrain.getState().Pose.getRotation().getRadians();

    SmartDashboard.putNumber("PoseErrorX", ErrorX);
    SmartDashboard.putNumber("PoseErrorY", ErrorY);

    double OutputH = MathUtil.clamp(ControllerH.calculate(ErrorH), -DriveConstants.MaxAngularRate, DriveConstants.MaxAngularRate);

    SmartDashboard.putNumber("PoseOutputH", OutputH);

    m_drivetrain.setControl(
      drive.withRotationalRate(OutputH)
    );
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
