package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

public class DynamicShootToGround extends Command {
  private final PIDController m_controllerH = DriveConstants.ControllerH;
  private final Debouncer m_aimDebouncer = new Debouncer(0.1);

  private CommandSwerveDrivetrain m_drivetrain;
  private LauncherRollers m_launcherRollers;
  private Conveyor m_conveyor;

  private boolean m_runonce = true;

  public DynamicShootToGround(CommandSwerveDrivetrain drivetrain, LauncherRollers rollers, Conveyor conveyor) {
    m_controllerH.enableContinuousInput(-180, 180);
    m_drivetrain = drivetrain;
    m_launcherRollers = rollers;
    m_conveyor = conveyor;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    SwerveDriveState state = m_drivetrain.getState();
    Pose2d currentPose = state.Pose;
    Pose2d targetPose = LiveDriveStats.DYNAMIC_SHOOT_TARGET;
    
    // 1. Calculate the vector to the target
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    double distToTarget = Math.hypot(dx, dy);

    // 2. Normalize the vector (Unit Vector)
    double unitX = dx / distToTarget;
    double unitY = dy / distToTarget;

    // 3. Get current velocities
    double vx = state.Speeds.vxMetersPerSecond;
    double vy = state.Speeds.vyMetersPerSecond;

    // 4. Calculate Radial Velocity (Towards target) and Tangential (Sideways)
    // Dot product for movement towards target
    double velocityTowardsTarget = (vx * unitX) + (vy * unitY); 
    // 2D Cross product magnitude for movement perpendicular to target
    double velocityPerpendicular = (vx * unitY) - (vy * unitX); 

    // 5. Apply "Jerryrigged" factors
    double adjustedDist = distToTarget - (velocityTowardsTarget * LauncherConstants.TargetConstants.distCoefficient);

    // Adjust heading error based on sideways drift
    double headingLead = 0;//velocityPerpendicular * adjustedDist * LauncherConstants.TargetConstants.leadCoefficient;

    // Original heading calculation
    double rawHeadingError = Math.toDegrees(Math.atan2(dy, dx) - state.Pose.getRotation().getRadians());
    // Add the lead compensation
    double headingError = MathUtil.inputModulus(rawHeadingError + headingLead, -180, 180);

    //Rotation
    double outputH = m_controllerH.calculate(headingError);
    outputH = MathUtil.clamp(outputH, -DriveConstants.MaxAngularRate*DriveConstants.goToPoseMaxspeeds, DriveConstants.MaxAngularRate*DriveConstants.goToPoseMaxspeeds);
    LiveDriveStats.OUTPUT_H = outputH;

    //Shooter
    double topShooterSpeed = LauncherConstants.TargetConstants.shooterHubInterpolator1.getInterpolatedValue(adjustedDist);
    double backShooterSpeed = LauncherConstants.TargetConstants.shooterHubInterpolator2.getInterpolatedValue(adjustedDist);

    if(LiveDriveStats.AUTO_SHOOTING){
      m_launcherRollers.setSpeedTop(topShooterSpeed);
      m_launcherRollers.setSpeedBack(backShooterSpeed);
    }

    //Shoot if Aligned on Target
    boolean isAligned = m_aimDebouncer.calculate(Math.abs(headingError) < CommandConstants.ShootAngleTolerance);
    if(LiveDriveStats.AUTO_SHOOTING){
      if(isAligned){
        m_conveyor.setTarget(ConveyorConstants.RUNNING_SPEED);
        m_launcherRollers.setSpeedBack(backShooterSpeed);

        if(m_runonce){
          m_launcherRollers.setSpeedBottom(LauncherConstants.OUTTAKE_SPEED_BOTTOM);
          m_runonce = false;
        }
      }
      else{
        m_conveyor.setTarget(0);
        m_launcherRollers.setSpeedBottom(0);
        m_runonce = true;
      }
    }

    // Telemetry
    LiveDriveStats.CURRENT_SHOOT_TARGET1 = targetPose;
    SmartDashboard.putNumber("DSTG: Distance to Hub", distToTarget);
    SmartDashboard.putBoolean("DSTG: Ready to Shoot", m_aimDebouncer.calculate(isAligned));
    SmartDashboard.putNumber("DSTG: Output H", outputH);
    SmartDashboard.putNumber("DSTG: Heading Error", headingError);
  }

  @Override public void end(boolean interrupted) {
    m_conveyor.setTarget(0);
    m_launcherRollers.setSpeedBottom(0);
     m_launcherRollers.setSpeedBack(0);
  }

  @Override public boolean isFinished() { return false; }
}