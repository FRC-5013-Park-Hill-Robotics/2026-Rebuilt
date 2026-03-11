// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.ConveyorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LiveDriveStats;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.trobot5013lib.AverageOverTime;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnAndShootToTarget extends Command {
  /** Creates a new TurnToPose. */

  private final PIDController ControllerH = DriveConstants.ControllerH;

  private final Debouncer mShootDebouncer = new Debouncer(0.2);

  private CommandSwerveDrivetrain m_drivetrain;
  private Pose2d m_targetPose;
  private LauncherRollers m_LauncherRollers;
  private Conveyor m_Conveyor;

  public TurnAndShootToTarget(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, LauncherRollers rollers, Conveyor conveyor) {
    ControllerH.enableContinuousInput(-Math.PI, Math.PI);
    m_drivetrain = drivetrain;
    m_targetPose = targetPose;
    m_LauncherRollers = rollers;
    m_Conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveDriveState state = m_drivetrain.getState();

    double VelocityX = state.Speeds.vxMetersPerSecond;
    double VelocityY = state.Speeds.vxMetersPerSecond;

    //Target Pose Offset by Robot Velocity
    double ErrorX = m_targetPose.getX() - state.Pose.getX();
    double ErrorY = m_targetPose.getX() - state.Pose.getY();

    double DistanceToTarget = Math.sqrt(Math.pow(ErrorX, 2) + Math.pow(ErrorY, 2));

    double ErrorXwV = ErrorX + DistanceToTarget*VelocityX*DriveConstants.DistanceAndVelocityCoefficient;
    double ErrorYwV = ErrorY + DistanceToTarget*VelocityY*DriveConstants.DistanceAndVelocityCoefficient;

    double ErrorH = Math.atan2(ErrorXwV, ErrorYwV) - state.Pose.getRotation().getRadians();

    //Drive
    double OutputH = MathUtil.clamp(ControllerH.calculate(ErrorH), -DriveConstants.MaxAngularRate, DriveConstants.MaxAngularRate);

    LiveDriveStats.OUTPUT_H = OutputH;

    //Prepare Top Shooter
    m_LauncherRollers.setSpeedTop(m_LauncherRollers.getSpeedFromDist(DistanceToTarget));
    
    //Shoot
    if(mShootDebouncer.calculate(Math.abs(Math.toDegrees(ErrorH)) < CommandConstants.ShootAngleTolerance)){
      m_Conveyor.setTarget(ConveyorConstants.RUNNING_SPEED);
      m_LauncherRollers.setSpeedBottom(LauncherConstants.OUTTAKE_SPEED_BOTTOM);
    }
    else{
      m_Conveyor.setTarget(0);
      m_LauncherRollers.setSpeedBottom(0);
    }

    //Print
    SmartDashboard.putNumber("TAS: PoseErrorX", ErrorX);
    SmartDashboard.putNumber("TAS: PoseErrorY", ErrorY);
    SmartDashboard.putNumber("TAS: PoseErrorH", ErrorH);
    SmartDashboard.putNumber("TAS: PoseErrorH (Degrees)", ErrorH);
    SmartDashboard.putBoolean("TAS: Angle Within Tollerance", Math.abs(Math.toDegrees(ErrorH)) < CommandConstants.ShootAngleTolerance);
    SmartDashboard.putNumber("TAS: OutputH", OutputH);
    LiveDriveStats.CURRENT_SHOOT_TARGET1 = new Pose2d(m_targetPose.getX(), m_targetPose.getY(), new Rotation2d(0));
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
