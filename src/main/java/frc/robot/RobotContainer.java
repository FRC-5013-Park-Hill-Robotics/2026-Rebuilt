// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DynamicShootToGround;
import frc.robot.commands.DynamicTargetPose;
import frc.robot.commands.GamepadDrive;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.TurnToPose;
import frc.robot.commands.UnlimitedGamepadDrive;
import frc.robot.constants.ConveyorConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LiveDriveStats;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.LauncherConstants.TargetConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.Vision;
import frc.robot.commands.TurnToPose;
import frc.robot.commands.TurnAndShootFromZones;
import frc.robot.commands.TurnAndShootFromZonesForAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController mDriver = new CommandXboxController(0);
  private final CommandXboxController mOpperator = new CommandXboxController(1);
  public final CommandSwerveDrivetrain mDrivetrain = TunerConstants.createDrivetrain();

  public final Intake mIntake = new Intake();
  public final Conveyor mConveyor = new Conveyor();
  public final Feeder mFeeder = new Feeder();
  public final LauncherRollers mRollers = new LauncherRollers();
  private static Alliance mAlliance = Alliance.Red;

  public static RobotContainer instance;

  private Field2d m_field = new Field2d();

  private final SendableChooser<Command> autoChooser;

  public final Vision camera = new Vision(mDrivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    instance = this;
    configureBindings();
    configureAutonomousCommands();

    SmartDashboard.putData("Field", m_field);

    // Build an auto chooser. This will use Commands.none() as the default option.
    SmartDashboard.clearPersistent("Auto Chooser");
    autoChooser = AutoBuilder.buildAutoChooser(); 
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    mDrivetrain.setDefaultCommand(new GamepadDrive(mDriver));

    mRollers.setDefaultCommand(new PrepareShooter(mDrivetrain, mRollers));

    mDriver.back().onTrue(mDrivetrain.runOnce(() -> mDrivetrain.seedFieldCentric()));
    mDriver.start().onTrue(mRollers.toggleStopCommand().alongWith(mConveyor.stopC()));
   
    mDriver.y().onTrue(mConveyor.setTargetC(ConveyorConstants.RUNNING_SPEED).alongWith(mFeeder.outtakeCommand()))
      .onFalse(mConveyor.setTargetC(0).alongWith(mFeeder.setSpeedCommand(0)));
    mDriver.x().whileTrue(new DynamicTargetPose(mDriver));
    // mDriver.a().onTrue(mIntake.toggleIntakePositionC()); 
    mDriver.b().onTrue(mIntake.moveIntakeInC()); 
    mDriver.a().onTrue(mIntake.moveIntakeOutC()); 


    mDriver.leftBumper().onTrue(mIntake.setTargetC(-IntakeConstants.intakeSpeed).alongWith(mConveyor.setTargetC(-ConveyorConstants.RUNNING_SPEED)).alongWith(mFeeder.disturbCommand()))
     .onFalse(mIntake.setTargetC(0).alongWith(mConveyor.setTargetC(0)).alongWith(mFeeder.setSpeedCommand(0)));
    mDriver.rightBumper().onTrue(mIntake.setTargetC(IntakeConstants.intakeSpeed))
     .onFalse(mIntake.setTargetC(0));

    mDriver.leftTrigger().whileTrue(new DynamicShootToGround(mDrivetrain, mRollers, mConveyor, mFeeder, mDriver));
    mDriver.rightTrigger().whileTrue(mConveyor.setTargetC(ConveyorConstants.RUNNING_SPEED).alongWith(mFeeder.outtakeCommand()).alongWith(mRollers.setSpeedCommand(LauncherConstants.FROM_TOWER_SPEED)))
      .onFalse(mConveyor.setTargetC(0).alongWith(mFeeder.setSpeedCommand(0)));
    
    // mDriver.povUp().onTrue(mRollers.incrementSpeedCommand(0.5));
    // mDriver.povDown().onTrue(mRollers.incrementSpeedCommand(-0.5));
    
    // mDriver.povLeft().onTrue(mRollers.incrementSpeedCommand(-5));
    // mDriver.povRight().onTrue(mRollers.incrementSpeedCommand(5));

    // mOpperator.leftTrigger().whileTrue(mRollers.setSpeedsCommand(LauncherConstants.SOLID_POSITION_TOP, LauncherConstants.SOLID_POSITION_BACK));
    // mOpperator.rightTrigger().onTrue(mConveyor.setTargetC(ConveyorConstants.RUNNING_SPEED).alongWith(mRollers.setSpeedBottomCommand(LauncherConstants.OUTTAKE_SPEED_BOTTOM)))
    //   .onFalse(mConveyor.setTargetC(0).alongWith(mRollers.setSpeedBottomCommand(0)));

    mOpperator.a().whileTrue(new DynamicTargetPose(mOpperator));
    mOpperator.b().onTrue(camera.toggleVisionUpdatesC());
    mOpperator.x().onTrue(mRollers.toggleStopCommand().alongWith(mConveyor.stopC()));
    mOpperator.y().onTrue(mRollers.toggleAutoShootingCommand());
  }
 
  public void updateField(){
    SmartDashboard.putBoolean("Auto Shooting Enabled", LiveDriveStats.AUTO_SHOOTING);

    Pose2d i = mDrivetrain.getState().Pose;
    m_field.setRobotPose(i);

    FieldObject2d targets = m_field.getObject("Targets");
    targets.setPoses(LiveDriveStats.CURRENT_SHOOT_TARGET1, LiveDriveStats.DYNAMIC_SHOOT_TARGET);

    SmartDashboard.putNumber("BotX", mDrivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("BotY", mDrivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("BotH", mDrivetrain.getState().Pose.getRotation().getDegrees());
  }


  public void configureAutonomousCommands() {
    WaitCommand wait5 = new WaitCommand(0.5);
    NamedCommands.registerCommand("Wait0.5", wait5);

    WaitCommand wait10 = new WaitCommand(1);
    NamedCommands.registerCommand("Wait1", wait10);

    WaitCommand wait15 = new WaitCommand(1.5);
    NamedCommands.registerCommand("Wait1.5", wait15);

    WaitCommand wait4 = new WaitCommand(4);
    NamedCommands.registerCommand("Wait4", wait4);

    // Command shootCommand = mConveyor.setTargetC(ConveyorConstants.RUNNING_SPEED).alongWith(mRollers.setSpeedBottomCommand(LauncherConstants.OUTTAKE_SPEED_BOTTOM));
    // NamedCommands.registerCommand("Shoot", shootCommand);

    // Command shootstopCommand = mConveyor.setTargetC(0).alongWith(mRollers.setSpeedBottomCommand(0));
    // NamedCommands.registerCom mand("ShootStop", shootstopCommand);

    NamedCommands.registerCommand("IntakeDownA", mIntake.moveIntakeOutC());
    NamedCommands.registerCommand("IntakeUp", mIntake.moveIntakeInC());
    NamedCommands.registerCommand("Intake", mIntake.setTargetC(IntakeConstants.intakeSpeed));
    NamedCommands.registerCommand("IntakeStop", mIntake.setTargetC(0));

    // NamedCommands.registerCommand("Const. Wheel Speed", new TurnAndShootFromZonesForAuto(mDrivetrain, mRollers, mConveyor, 4));
    // Command shootCommand = mConveyor.setTargetC(ConveyorConstants.RUNNING_SPEED).alongWith(mRollers.setSpeedBottomCommand(LauncherConstants.OUTTAKE_SPEED_BOTTOM));
    // NamedCommands.registerCommand("Shoot", shootCommand);

    NamedCommands.registerCommand("Lock and Shoot 4 Sec", new TurnAndShootFromZonesForAuto(mDrivetrain, mRollers, mConveyor, mFeeder, 15));
  
    NamedCommands.registerCommand("Prepare Shooter", new PrepareShooter(mDrivetrain, mRollers));
  }
  
  public Command getAutonomousCommand() {
    return new Command() {};
    //return autoChooser.getSelected();
  }

  public static void setAlliance(Alliance alliance){
    mAlliance = alliance;
  }

  public static Alliance getAlliance(){
    return mAlliance;
  }
  
  public static RobotContainer getInstance(){
		return instance;
	}

  public CommandSwerveDrivetrain getDrivetrain(){
    return mDrivetrain;
  }
}
