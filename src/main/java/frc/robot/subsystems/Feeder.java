// Copybottom (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CANConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LiveDriveStats;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class Feeder extends SubsystemBase {
  /** Creates a new LauncherRollers. */
  private TalonFX mFeederMotor = new TalonFX(CANConstants.FEEDER_ID, CANConstants.CANBUS_AUX);
  
  private SlewRateLimiter m_FeederLimiter = new SlewRateLimiter(300);
  private VelocityVoltage m_FeederVoltage = new VelocityVoltage(0);

  private double goalSpeedFeeder = 0;

  public Feeder() {
    TalonFXConfiguration Config = new TalonFXConfiguration();
    Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    Config.Slot0.kP = FeederConstants.RollerGains.kP;
    Config.Slot0.kI = LauncherConstants.RollerGains.kI;
    Config.Slot0.kD = LauncherConstants.RollerGains.kD;
    Config.Slot0.kS = LauncherConstants.RollerGains.kS;
    Config.Slot0.kV = LauncherConstants.RollerGains.kV;
    Config.Slot0.kA = LauncherConstants.RollerGains.kA;
    Config.CurrentLimits.StatorCurrentLimit = 50;
    Config.CurrentLimits.SupplyCurrentLimit = 40;
    mFeederMotor.set(0);
    mFeederMotor.getConfigurator().apply(Config);
    m_FeederVoltage.withSlot(0);
  }

  @Override
  public void periodic() {
    
    m_FeederVoltage.withVelocity(m_FeederLimiter.calculate(goalSpeedFeeder));
    mFeederMotor.setControl(m_FeederVoltage);
    
    SmartDashboard.putNumber("Feeder: Speed", mFeederMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Feeder: Goal Speed", goalSpeedFeeder);
  }

  // FEEDER METHODS
  public void setSpeed(double rps) {
    this.goalSpeedFeeder = rps;
  }

  public void incrementSpeed(double rpsChange) {
    this.goalSpeedFeeder += rpsChange;
  }

  public void outtake(){
    this.goalSpeedFeeder = FeederConstants.OUTTAKE_SPEED;
  }

  public Command incrementSpeedCommand(double rpsChange){
    Command result = runOnce(()-> incrementSpeed(rpsChange));
    return result;
  } 
  
  public Command setSpeedCommand(double rps){
    Command result = runOnce(()-> setSpeed(rps));
    return result;
  } 

  public Command outtakeCommand(){
    Command result = runOnce(()-> outtake());
    return result;
  } 

  // // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // // Mutable holder for unit-safe linear distance values, persisted to avoid
  // // reallocation.
  // private final MutableMeasure<Angle> m_rotation = mutable(Rotations.of(0));
  // // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // // reallocation.
  // private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  // private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
  //         // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
  //         new SysIdRoutine.Config( Volts.of(1).per(Seconds.of(1)), Volts.of(7), null,null),
  //         new SysIdRoutine.Mechanism(
  //                 // Tell SysId how to plumb the driving voltage to the motors.
  //                 (Measure<Voltage> volts) -> {
  //                     bottomMotor.setVoltage(volts.in(Volts));
  //                     topMotor.setVoltage(volts.in(Volts));
  //                 },
  //                 // Tell SysId how to record a frame of data for each motor on the mechanism
  //                 // being
  //                 // characterized.
  //                 log -> {
  //                     // Record a frame for the wheel motor. 
  //                     log.motor("bottom")
  //                             .voltage(
  //                                     m_appliedVoltage.mut_replace(
  //                                             bottomMotor.get() * RobotController.getBatteryVoltage(), Volts))
  //                             .angularPosition(m_rotation.mut_replace(bottomMotor.getPosition().getValueAsDouble(), Rotations))
  //                             .angularVelocity(
  //                                     m_velocity.mut_replace(bottomMotor.getVelocity().getValueAsDouble(), RadiansPerSecond));
  //                     log.motor("top")
  //                             .voltage(
  //                                     m_appliedVoltage.mut_replace(
  //                                             topMotor.get() * RobotController.getBatteryVoltage(), Volts))
  //                             .angularPosition(m_rotation.mut_replace(topMotor.getPosition().getValueAsDouble(), Rotations))
  //                             .angularVelocity(
  //                                     m_velocity.mut_replace(topMotor.getVelocity().getValueAsDouble(), RadiansPerSecond));

  //                 },
  //                 // Tell SysId to make generated commands require this subsystem, suffix test
  //                 // state in
  //                 // WPILog with this subsystem's name ("LauncherRollers")
  //                 this));

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //     return m_sysIdRoutine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //     return m_sysIdRoutine.dynamic(direction);
  // }
}