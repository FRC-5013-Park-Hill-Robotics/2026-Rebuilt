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
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LiveDriveStats;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class LauncherRollers extends SubsystemBase {
  /** Creates a new LauncherRollers. */
  private TalonFX OuttakeLT = new TalonFX(CANConstants.OUTTAKE_LT_ID, CANConstants.CANBUS_AUX);
  private TalonFX OuttakeLB= new TalonFX(CANConstants.OUTTAKE_LB_ID, CANConstants.CANBUS_AUX);
  private TalonFX OuttakeRT= new TalonFX(CANConstants.OUTTAKE_RT_ID, CANConstants.CANBUS_AUX);

  private SlewRateLimiter m_Limiter = new SlewRateLimiter(80);
  private VelocityVoltage m_Voltage = new VelocityVoltage(0);

  private double goalSpeed = 0;
  private boolean STOP = true;

  private boolean RevWheels = false; //When outtaking, spin up wheels a little bit
  private Timer RevTimer = new Timer();

  public LauncherRollers() {
    TalonFXConfiguration topConfig1 = new TalonFXConfiguration();
    topConfig1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topConfig1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topConfig1.Slot0.kP = LauncherConstants.RollerGains.kP;
    topConfig1.Slot0.kI = LauncherConstants.RollerGains.kI;
    topConfig1.Slot0.kD = LauncherConstants.RollerGains.kD;
    topConfig1.Slot0.kS = LauncherConstants.RollerGains.kS;
    topConfig1.Slot0.kV = LauncherConstants.RollerGains.kV;
    topConfig1.Slot0.kA = LauncherConstants.RollerGains.kA;
    topConfig1.CurrentLimits.StatorCurrentLimit = 60;
    topConfig1.CurrentLimits.SupplyCurrentLimit = 40;
    OuttakeLT.set(0);
    OuttakeLB.set(0);
    OuttakeLT.getConfigurator().apply(topConfig1);
    OuttakeLB.getConfigurator().apply(topConfig1);
    // OuttakeLT.setControl(new Follower(CANConstants.OUTTAKE_RT_ID, MotorAlignmentValue.Aligned));
    // OuttakeLB.setControl(new Follower(CANConstants.OUTTAKE_RT_ID, MotorAlignmentValue.Aligned));

    TalonFXConfiguration topConfig2 = new TalonFXConfiguration();
    topConfig2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    topConfig2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topConfig2.Slot0.kP = LauncherConstants.RollerGains.kP;
    topConfig2.Slot0.kI = LauncherConstants.RollerGains.kI;
    topConfig2.Slot0.kD = LauncherConstants.RollerGains.kD;
    topConfig2.Slot0.kS = LauncherConstants.RollerGains.kS;
    topConfig2.Slot0.kV = LauncherConstants.RollerGains.kV;
    topConfig2.Slot0.kA = LauncherConstants.RollerGains.kA;
    topConfig2.CurrentLimits.StatorCurrentLimit = 60;
    topConfig2.CurrentLimits.SupplyCurrentLimit = 40;
    OuttakeRT.set(0);
    OuttakeRT.getConfigurator().apply(topConfig2);
    m_Voltage.withSlot(0);

    RevTimer.start();

    this.start();
  }

  @Override
  public void periodic() {
    
    if(!STOP){
      m_Voltage.withVelocity(m_Limiter.calculate(goalSpeed));
      OuttakeRT.setControl(m_Voltage);
      // Followers
      OuttakeLB.setControl(m_Voltage);
      OuttakeLT.setControl(m_Voltage);
    }

    // if(RevWheels && !RevTimer.hasElapsed(LauncherConstants.REV_TIME)){
    //   m_TopVoltage.withVelocity(m_TopLimiter.calculate(goalSpeedTop+LauncherConstants.REV_OFFSET));
    //   OuttakeLT.setControl(m_TopVoltage);
    //   OuttakeRB.setControl(m_TopVoltage);
    //   OuttakeRT.setControl(m_TopVoltage);

    //   m_BackVoltage.withVelocity(m_BackLimiter.calculate(goalSpeedBack+LauncherConstants.REV_OFFSET));
    //   OuttakeLB.setControl(m_BackVoltage);
    // }

    if(goalSpeed == 0){
      OuttakeLT.setVoltage(0);
      OuttakeLB.setVoltage(0);
      OuttakeRT.setVoltage(0);
    }
    
    SmartDashboard.putNumber("Outtake: LT Speed", OuttakeLT.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Outtake: RB Speed", OuttakeLB.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Outtake: RT Speed", OuttakeRT.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Outtake: Top Goal", goalSpeed);
  }

  // GENERAL METHODS
  // public double getSpeedFromDist(double distance) {
  //   return LauncherConstants.TargetConstants.getShooterSpeed(distance, 0);
  // } //No Command Attached

  // GENERAL COMMANDS
  public void start() {
    STOP = false;
    this.goalSpeed = 0;
  }
  
  public void stopLauncher(){
    STOP = true;
    this.goalSpeed = 0;
  } 

  public void toggleStopLauncher(){
    STOP = !STOP;
    this.goalSpeed = 0;
  } 

  public void toggleAutoShooting(){
    LiveDriveStats.AUTO_SHOOTING = !LiveDriveStats.AUTO_SHOOTING;
  }

  public Command startCommand(){
    Command result = runOnce(this::start);
    return result;
  }

  public Command stopCommand(){
    Command result = runOnce(this::stopLauncher);
    return result;
  }

  public Command toggleStopCommand(){
    Command result = runOnce(this::toggleStopLauncher);
    return result;
  }

  public Command toggleAutoShootingCommand(){
    Command result = runOnce(()->toggleAutoShooting());
    return result;
  }

  //public Command setSpeedCommand( SpeedTop,)

  // TOP ROLLER METHODS
  public void incrementSpeed(double rpsChange) {
    this.goalSpeed += rpsChange;
  }

  public void setSpeed(double rps) {
    this.goalSpeed = rps;
  }


  public Command incrementSpeedCommand(double rpsChange){
    Command result = runOnce(()-> incrementSpeed(rpsChange));
    return result;
  } 

  public Command setSpeedCommand(double rps){
    Command result = runOnce(()-> setSpeed(rps));
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