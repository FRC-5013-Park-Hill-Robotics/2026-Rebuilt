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
import frc.robot.constants.ConveyorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LiveDriveStats;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.trobot5013lib.ShooterCalculator;
import frc.robot.trobot5013lib.ShooterCalculator.ShotData;

public class TurnAndShootFromZones extends Command {
  private final PIDController m_controllerH = DriveConstants.ControllerH;
  private final Debouncer m_aimDebouncer = new Debouncer(0.1);

  private CommandSwerveDrivetrain m_drivetrain;
  private LauncherRollers m_launcherRollers;
  private Conveyor m_Conveyor;
  private Alliance m_alliance;

  public TurnAndShootFromZones(CommandSwerveDrivetrain drivetrain, LauncherRollers rollers, Conveyor conveyor) {
    m_controllerH.enableContinuousInput(-Math.PI, Math.PI);
    m_drivetrain = drivetrain;
    m_launcherRollers = rollers;
    m_Conveyor = conveyor;
    addRequirements(drivetrain, rollers);
  }

  @Override
  public void initialize() {
    m_alliance = RobotContainer.getAlliance();
  }

  @Override
  public void execute() {
    SwerveDriveState state = m_drivetrain.getState();
    Pose2d currentPose = state.Pose;
    Pose2d targetPose;

    // 1. Zone logic
    if (m_alliance == Alliance.Blue) {
      if (currentPose.getX() < PoseConstants.X_BLUE_SHUTTLE_CUTOFF) {
        targetPose = PoseConstants.BLUE_HUB;
      } else {
        targetPose = currentPose.nearest(PoseConstants.BLUE_SHUTTLE_POSES);
      }
    } else { 
      if (currentPose.getX() > PoseConstants.X_RED_SHUTTLE_CUTOFF) {
        targetPose = PoseConstants.RED_HUB;
      } else {
        targetPose = currentPose.nearest(PoseConstants.RED_SHUTTLE_POSES);
      }
    }

    ShotData data = ShooterCalculator.calculateShot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, currentPose, targetPose, LauncherConstants.TargetConstants.SHOOTER_DATA);

    //Rotation
    double outputH = m_controllerH.calculate(data.headingError.getDegrees());
    outputH = MathUtil.clamp(outputH, -DriveConstants.MaxAngularRate, DriveConstants.MaxAngularRate);
    LiveDriveStats.OUTPUT_H = outputH;

    //Shooter
    m_launcherRollers.setSpeedTop(data.rollerSpeed);

    //Shoot if Aligned on Target
    boolean isAligned = Math.abs(m_controllerH.getPositionError()) < CommandConstants.ShootAngleTolerance;
    if(isAligned){
      m_Conveyor.setTarget(ConveyorConstants.RUNNING_SPEED);
      m_launcherRollers.setSpeedBottom(LauncherConstants.OUTTAKE_SPEED_BOTTOM);
    }
    else{
      m_Conveyor.setTarget(0);
      m_launcherRollers.setSpeedBottom(0);
    }

    // Telemetry
    SmartDashboard.putBoolean("TASFZ: Ready to Shoot", m_aimDebouncer.calculate(isAligned));
    SmartDashboard.putNumber("TASFZ: Output H", outputH);
    SmartDashboard.putNumber("TASFZ: Heading Error", data.headingError.getDegrees());
    SmartDashboard.putNumber("TASFZ: RollerSpeed", data.rollerSpeed);
  }

  @Override public void end(boolean interrupted) { m_drivetrain.setControl(new SwerveRequest.Idle()); }
  @Override public boolean isFinished() { return false; }
}