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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.ConveyorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LiveDriveStats;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.trobot5013lib.AverageOverTime;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFromZones extends Command {
  /** Creates a new TurnToPose. */

  private final PIDController ControllerH = DriveConstants.ControllerH;

  private CommandSwerveDrivetrain m_drivetrain;
  private Pose2d m_targetPose;
  private LauncherRollers m_LauncherRollers;

  private Alliance m_Alliance;
  public enum Mode {
    Shuttle,
    Hub
  }
  private Mode m_Mode;

  public ShootFromZones(CommandSwerveDrivetrain drivetrain, LauncherRollers rollers) {
    ControllerH.enableContinuousInput(-Math.PI, Math.PI);
    m_drivetrain = drivetrain;
    m_targetPose = new Pose2d();
    m_LauncherRollers = rollers;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Alliance = RobotContainer.getAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SwerveDriveState state = m_drivetrain.getState();

    if(m_Alliance == Alliance.Blue){
      if(state.Pose.getX() < PoseConstants.X_BLUE_SHUTTLE_CUTOFF){
        m_targetPose = PoseConstants.BLUE_HUB;
        m_Mode = Mode.Hub;
      }
      else{
        m_targetPose = state.Pose.nearest(PoseConstants.BLUE_SHUTTLE_POSES);
         m_Mode = Mode.Shuttle;
      }
    }
    if(m_Alliance == Alliance.Red){
      if(state.Pose.getX() > PoseConstants.X_RED_SHUTTLE_CUTOFF){
        m_targetPose = PoseConstants.RED_HUB;
        m_Mode = Mode.Hub;
      }
      else{
        m_targetPose = state.Pose.nearest(PoseConstants.RED_SHUTTLE_POSES);
        m_Mode = Mode.Shuttle;
      }
    }

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

    //Print
    SmartDashboard.putNumber("SFZ: PoseErrorX", ErrorX);
    SmartDashboard.putNumber("SFZ: PoseErrorY", ErrorY);
    SmartDashboard.putNumber("SFZ: PoseErrorH", ErrorH);
    SmartDashboard.putBoolean("SFZ: Angle Within Tollerance", Math.abs(Math.toDegrees(ErrorH)) < CommandConstants.ShootAngleTolerance);
    SmartDashboard.putNumber("SFZ: OutputH", OutputH);
    LiveDriveStats.CURRENT_SHOOT_TARGET1 = new Pose2d(m_targetPose.getX(), m_targetPose.getY(), new Rotation2d(0));
    LiveDriveStats.CURRENT_SHOOT_TARGET2 = new Pose2d(m_targetPose.getX()+DistanceToTarget*VelocityX*DriveConstants.DistanceAndVelocityCoefficient, m_targetPose.getY()+DistanceToTarget*VelocityY*DriveConstants.DistanceAndVelocityCoefficient, new Rotation2d(0));
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
