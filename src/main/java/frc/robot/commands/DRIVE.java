// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LiveDriveStats;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DRIVE extends Command {

  private CommandSwerveDrivetrain m_drivetrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.movementLimitAmount);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.movementLimitAmount);
	private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  /** Creates a new DRIVE. */
  public DRIVE() {
    m_drivetrain = RobotContainer.getInstance().getDrivetrain();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setControl(drive
			.withVelocityX(-CommandSwerveDrivetrain.percentOutputToMetersPerSecond(xLimiter.calculate(LiveDriveStats.OUTPUT_X)))
			.withVelocityY(CommandSwerveDrivetrain.percentOutputToMetersPerSecond(yLimiter.calculate(LiveDriveStats.OUTPUT_Y))) 
			.withRotationalRate(-CommandSwerveDrivetrain.percentOutputToRadiansPerSecond(rotationLimiter.calculate(LiveDriveStats.OUTPUT_H))));
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
