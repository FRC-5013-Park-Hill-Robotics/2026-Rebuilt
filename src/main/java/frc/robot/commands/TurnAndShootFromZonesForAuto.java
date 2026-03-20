package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
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

public class TurnAndShootFromZonesForAuto extends Command {
  private final PIDController m_controllerH = DriveConstants.ControllerH;
  private final Debouncer m_aimDebouncer = new Debouncer(0.1);

  private CommandSwerveDrivetrain m_drivetrain;
  private LauncherRollers m_launcherRollers;
  private Conveyor m_conveyor;
  private Alliance m_alliance;

  private double m_shootTime;
  private Timer m_Timer = new Timer();
  
  private boolean m_runonce = true;

  public TurnAndShootFromZonesForAuto(CommandSwerveDrivetrain drivetrain, LauncherRollers rollers, Conveyor conveyor, double ShootTime) {
    m_controllerH.enableContinuousInput(-180, 180);
    m_drivetrain = drivetrain;
    m_launcherRollers = rollers;
    m_conveyor = conveyor;
  }

  @Override
  public void initialize() {
    m_alliance = RobotContainer.getAlliance();
    m_Timer.reset();
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

    // 6. Calculate Shooter Speed and Heading with offsets
    double backShooterSpeed = LauncherConstants.TargetConstants.shooterHubInterpolator2.getInterpolatedValue(adjustedDist);

    // Original heading calculation
    double rawHeadingError = Math.toDegrees(Math.atan2(dy, dx) - state.Pose.getRotation().getRadians());
    // Add the lead compensation
    double headingError = MathUtil.inputModulus(rawHeadingError + headingLead, -180, 180);

    //Rotation
    double outputH = m_controllerH.calculate(headingError);
    outputH = MathUtil.clamp(outputH, -DriveConstants.MaxAngularRate*DriveConstants.goToPoseMaxspeeds, DriveConstants.MaxAngularRate*DriveConstants.goToPoseMaxspeeds);
    LiveDriveStats.OUTPUT_H = outputH;

    //Shooter
    //m_launcherRollers.setSpeedBottom(bottomShooterSpeed);

    //Shoot if Aligned on Target
    boolean isAligned = (Math.abs(headingError) < CommandConstants.ShootAngleTolerance);
    if(LiveDriveStats.AUTO_SHOOTING){
      if(isAligned){
        m_conveyor.setTarget(ConveyorConstants.RUNNING_SPEED);
        m_launcherRollers.setSpeedBack(backShooterSpeed);

        if(m_runonce){
          m_launcherRollers.outtake();
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
    SmartDashboard.putNumber("TASFZ: Distance to Hub", distToTarget);
    SmartDashboard.putBoolean("TASFZ: Ready to Shoot", isAligned);
    SmartDashboard.putNumber("TASFZ: Output H", outputH);
    SmartDashboard.putNumber("TASFZ: Heading Error", headingError);
  }

  @Override public void end(boolean interrupted) {
    m_conveyor.setTarget(0);
    m_launcherRollers.setSpeedBottom(0);
    m_launcherRollers.setSpeedBack(0);
  }

  @Override public boolean isFinished() { 
    return !m_Timer.hasElapsed(m_shootTime); 
  }
}