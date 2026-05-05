package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LiveDriveStats;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.trobot5013lib.ShooterCalculator;
import frc.robot.trobot5013lib.ShooterCalculator.ShotData;

public class PrepareShooter extends Command {
  private CommandSwerveDrivetrain m_drivetrain;
  private LauncherRollers m_launcherRollers;
  private Alliance m_alliance;

  public PrepareShooter(CommandSwerveDrivetrain drivetrain, LauncherRollers rollers) {
    m_drivetrain = drivetrain;
    m_launcherRollers = rollers;

    addRequirements(rollers);
  }

  @Override
  public void initialize() {
    m_alliance = RobotContainer.getAlliance();
  }

  @Override
  public void execute() {
    SwerveDriveState state = m_drivetrain.getState();
    Pose2d currentPose = state.Pose;
    Pose2d targetPose = LiveDriveStats.DYNAMIC_SHOOT_TARGET;

    // 1. Zone logic
    // if (m_alliance == Alliance.Blue) {
    //   if (currentPose.getX() < PoseConstants.X_BLUE_SHUTTLE_CUTOFF) {
    //     targetPose = PoseConstants.BLUE_HUB;
    //   } else {
    //     targetPose = currentPose.nearest(PoseConstants.BLUE_SHUTTLE_POSES);
    //   }
    // } else { 
    //   if (currentPose.getX() > PoseConstants.X_RED_SHUTTLE_CUTOFF) {
    //     targetPose = PoseConstants.RED_HUB;
    //   } else {
    //     targetPose = currentPose.nearest(PoseConstants.RED_SHUTTLE_POSES);
    //   }
    // }

    //ShotData data = ShooterCalculator.calculateShot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, currentPose, targetPose, LauncherConstants.TargetConstants.SHOOTER_DATA);
    
    double distToTarget = Math.hypot(targetPose.getX()-state.Pose.getX(), targetPose.getY()-state.Pose.getY());
    double shooterSpeed = LauncherConstants.TargetConstants.shooterGroundInterpolator1.getInterpolatedValue(distToTarget);

    // Rollers
    if(LiveDriveStats.AUTO_SHOOTING){
      m_launcherRollers.setSpeed(shooterSpeed);
    }

    SmartDashboard.putNumber("PS: Distance to Hub", distToTarget);
    SmartDashboard.putNumber("PS: Top Shooter Speed", shooterSpeed);
    LiveDriveStats.CURRENT_SHOOT_TARGET1 = targetPose;
  }

  @Override public void end(boolean interrupted) {}

  @Override public boolean isFinished() { return false; }
}