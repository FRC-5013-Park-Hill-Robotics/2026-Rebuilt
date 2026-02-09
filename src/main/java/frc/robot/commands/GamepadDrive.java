// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GamepadDrive extends Command {
	private CommandSwerveDrivetrain m_drivetrain;
	private CommandXboxController m_gamepad;
	private SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.movementLimitAmount);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.movementLimitAmount);
	private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	//private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	/**
	 * Constructor method for the GamepadDrive class
	 * - Creates a new GamepadDrive object.
	 */
	public GamepadDrive(CommandXboxController gamepad) {
		super();
		m_gamepad = gamepad;
		m_drivetrain = RobotContainer.getInstance().getDrivetrain();
		addRequirements(m_drivetrain);
	}

	@Override
	public void execute() {

		//double throttle = modifyAxis(m_gamepad.getRightTriggerAxis());
		double throttle = 1;

		double translationX = modifyAxis(-m_gamepad.getLeftY());
		double translationY = modifyAxis(-m_gamepad.getLeftX());
		double translationH = rotationLimiter.calculate(m_gamepad.getRightX()*0.75);
		
		if(!(translationX == 0.0 && translationY == 0.0)) {
			double angle = calculateTranslationDirection(translationX, translationY);
			translationX = Math.cos(angle) * throttle;
			translationY = Math.sin(angle) * throttle;
		}

		if(m_gamepad.getLeftTriggerAxis() > 0.5){
			translationX = translationX/3;
			translationY = translationY/3;
			translationH = translationH/2;
		}

		double i = 1.0;

		if(m_gamepad.getLeftTriggerAxis() > 0.5){
			i = 0.2;
		}

		m_drivetrain.setControl(drive
			.withVelocityX(-CommandSwerveDrivetrain.percentOutputToMetersPerSecond(xLimiter.calculate(i*translationX)))
			.withVelocityY(CommandSwerveDrivetrain.percentOutputToMetersPerSecond(yLimiter.calculate(i*translationY))) 
			.withRotationalRate(-CommandSwerveDrivetrain.percentOutputToRadiansPerSecond(translationH)));
		

		SmartDashboard.putNumber("Throttle", throttle);
		SmartDashboard.putNumber("Drive Rotation",-CommandSwerveDrivetrain.percentOutputToRadiansPerSecond(m_gamepad.getRightX()) );
		SmartDashboard.putNumber("VX", CommandSwerveDrivetrain.percentOutputToMetersPerSecond(xLimiter.calculate(translationX)));
		SmartDashboard.putNumber("VY", CommandSwerveDrivetrain.percentOutputToMetersPerSecond(yLimiter.calculate(translationY)));
		
	 }

	@Override
	public void end(boolean interrupted) {
		m_drivetrain.applyRequest(() -> brake);
	}

	private static double modifyAxis(double value) {
	
		return modifyAxis(value, 1);
	}
	private static double modifyAxis(double value, int exponent) {
		// Deadband
		value = MathUtil.applyDeadband(value, 0.1);

		 value = Math.copySign(Math.pow(value, exponent), value);

		return value;
	}
	
	private double calculateTranslationDirection(double x, double y) {
		// Calculate the angle.
		// Swapping x/y
		return Math.atan2(x, y) + Math.PI / 2;
	}

	
}