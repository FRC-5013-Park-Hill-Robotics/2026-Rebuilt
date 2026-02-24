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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DRIVE;
import frc.robot.commands.GamepadDrive;
import frc.robot.commands.TurnToPose;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.Vision;
import frc.robot.commands.TurnToPose;
import frc.robot.commands.Shuttle;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController mDriver = new CommandXboxController(0);
  public final CommandSwerveDrivetrain mDrivetrain = TunerConstants.createDrivetrain();

  public final Intake mIntake = new Intake();
  public final Conveyor mConveyor = new Conveyor();
  public final LauncherRollers mRollers = new LauncherRollers();
  private static Alliance mAlliance = Alliance.Red;

  public static RobotContainer instance;

  private Field2d m_field = new Field2d();

  private final SendableChooser<Command> autoChooser;

  //public final Vision camera = new Vision(mDrivetrain);

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

    mDriver.back().onTrue(mDrivetrain.runOnce(() -> mDrivetrain.seedFieldCentric()));
   
    mDriver.y().onTrue(mRollers.stopCommand().alongWith(mConveyor.stopC())); 

    mDriver.x().onTrue(mIntake.setTargetC(100))
     .onFalse(mIntake.setTargetC(0));

    mDriver.b().onTrue(mRollers.setSpeedTopCommand(20)
                          .andThen(mRollers.setSpeedBackCommand(20)));
    mDriver.a().onTrue(mRollers.setSpeedBottomCommand(15)
                          .alongWith(mConveyor.setTargetC(15)));

    mDriver.leftBumper().whileTrue(new TurnToPose(PoseConstants.BLUE_HUB, mDrivetrain)); 
    mDriver.rightBumper().whileTrue(new TurnToPose(PoseConstants.RED_HUB, mDrivetrain)); 

    mDriver.rightStick().whileTrue(new Shuttle(mDrivetrain));

    mDriver.povUp().onTrue(mRollers.incrementSpeedTopCommand(4));
    mDriver.povDown().onTrue(mRollers.incrementSpeedTopCommand(-4));
    
    // mDriver.povLeft().onTrue(mRollers.incrementSpeedBackCommand(-4));
    // mDriver.povRight().onTrue(mRollers.incrementSpeedBackCommand(4));
  }
 
  public void updateField(){
    Pose2d i = mDrivetrain.getState().Pose;
    m_field.setRobotPose(i);

    SmartDashboard.putNumber("BotX", mDrivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("BotY", mDrivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("BotH", mDrivetrain.getState().Pose.getRotation().getDegrees());
  }


  public void configureAutonomousCommands() {
    // WaitCommand wait025 = new WaitCommand(0.25);
    // NamedCommands.registerCommand("Wait0.25", wait025);

  
    WaitCommand wait5 = new WaitCommand(0.5);
    NamedCommands.registerCommand("Wait0.5", wait5);

    WaitCommand wait10 = new WaitCommand(1);
    NamedCommands.registerCommand("Wait1", wait10);

    WaitCommand wait15 = new WaitCommand(1.5);
    NamedCommands.registerCommand("Wait1.5", wait15);
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
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
