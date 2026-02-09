// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.GamepadDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController mDriver = new CommandXboxController(0);
  public final CommandSwerveDrivetrain mDrivetrain = TunerConstants.createDrivetrain();

  // public final Intake mIntake = new Intake();
  // public final LauncherRollers mRollers = new LauncherRollers();

  public static RobotContainer instance;

  private Field2d m_field = new Field2d();

  //public final Vision camera = new Vision(mDrivetrain);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    SmartDashboard.putData("Field", m_field);

    instance = this;
    configureBindings();
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
   
    // mDriver.x().onTrue(mIntake.setTargetC(100))
    // .onFalse(mIntake.setTargetC(0));

    // mDriver.b().onTrue(mIntake.setTargetC(-100))
    // .onFalse(mIntake.setTargetC(0));

    // mDriver.a().onTrue(mRollers.stopCommand()); 

    // mDriver.povUp().onTrue(mRollers.incrementSpeedTopCommand(4));
    // mDriver.povDown().onTrue(mRollers.incrementSpeedTopCommand(-4));
    
    // mDriver.povLeft().onTrue(mRollers.incrementSpeedBottomCommand(-4));
    // mDriver.povRight().onTrue(mRollers.incrementSpeedBottomCommand(4));
  }
 
  public void updateField(){
    Pose2d i = mDrivetrain.getState().Pose;
    m_field.setRobotPose(i);

    SmartDashboard.putNumber("BotX", mDrivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("BotY", mDrivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("BotH", mDrivetrain.getState().Pose.getRotation().getDegrees());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public static RobotContainer getInstance(){
		return instance;
	}

  public CommandSwerveDrivetrain getDrivetrain(){
    return mDrivetrain;
  }
}
