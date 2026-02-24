// Copybottom (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;

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
  private TalonFX OuttakeRB= new TalonFX(CANConstants.OUTTAKE_RB_ID, CANConstants.CANBUS_AUX);
  private TalonFX OuttakeBottom = new TalonFX(CANConstants.OUTTAKE_BOTTOM_ID, CANConstants.CANBUS_AUX);
  
  private VelocityVoltage m_BottomVoltage = new VelocityVoltage(0);
  private VelocityVoltage m_TopVoltage = new VelocityVoltage(0);
  private VelocityVoltage m_BackVoltage = new VelocityVoltage(0);
  private double goalSpeedBottom = 0;
  private double goalSpeedTop = 0;
  private double goalSpeedBack = 0;

  public LauncherRollers() {
    TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
    bottomConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomConfig.Slot0.kP = LauncherConstants.RollerGains.kP;
    bottomConfig.Slot0.kI = LauncherConstants.RollerGains.kI;
    bottomConfig.Slot0.kD = LauncherConstants.RollerGains.kD;
    bottomConfig.Slot0.kS = LauncherConstants.RollerGains.kS;
    bottomConfig.Slot0.kV = LauncherConstants.RollerGains.kV;
    bottomConfig.Slot0.kA = LauncherConstants.RollerGains.kA;
    OuttakeBottom.set(0);
    OuttakeBottom.getConfigurator().apply(bottomConfig);
    m_BottomVoltage.withSlot(0);
    
    TalonFXConfiguration topConfig1 = new TalonFXConfiguration();
    topConfig1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    topConfig1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topConfig1.Slot0.kP = LauncherConstants.RollerGains.kP;
    topConfig1.Slot0.kI = LauncherConstants.RollerGains.kI;
    topConfig1.Slot0.kD = LauncherConstants.RollerGains.kD;
    topConfig1.Slot0.kS = LauncherConstants.RollerGains.kS;
    topConfig1.Slot0.kV = LauncherConstants.RollerGains.kV;
    topConfig1.Slot0.kA = LauncherConstants.RollerGains.kA;
    OuttakeRT.set(0);
    OuttakeRB.set(0);
    OuttakeRT.getConfigurator().apply(topConfig1);
    OuttakeRB.getConfigurator().apply(topConfig1);
    m_TopVoltage.withSlot(0);

    TalonFXConfiguration topConfig2 = new TalonFXConfiguration();
    topConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topConfig2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topConfig2.Slot0.kP = LauncherConstants.RollerGains.kP;
    topConfig2.Slot0.kI = LauncherConstants.RollerGains.kI;
    topConfig2.Slot0.kD = LauncherConstants.RollerGains.kD;
    topConfig2.Slot0.kS = LauncherConstants.RollerGains.kS;
    topConfig2.Slot0.kV = LauncherConstants.RollerGains.kV;
    topConfig2.Slot0.kA = LauncherConstants.RollerGains.kA;
    OuttakeLT.set(0);
    OuttakeLT.getConfigurator().apply(topConfig2);
    m_TopVoltage.withSlot(0);

    TalonFXConfiguration backConfig = new TalonFXConfiguration();
    backConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    backConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    backConfig.Slot0.kP = LauncherConstants.RollerGains.kP;
    backConfig.Slot0.kI = LauncherConstants.RollerGains.kI;
    backConfig.Slot0.kD = LauncherConstants.RollerGains.kD;
    backConfig.Slot0.kS = LauncherConstants.RollerGains.kS;
    backConfig.Slot0.kV = LauncherConstants.RollerGains.kV;
    backConfig.Slot0.kA = LauncherConstants.RollerGains.kA;
    OuttakeLB.set(0);
    OuttakeLB.getConfigurator().apply(backConfig);
    m_BottomVoltage.withSlot(0);
    this.start();
  }

  @Override
  public void periodic() {
    
    m_BottomVoltage.withVelocity(goalSpeedBottom);
    OuttakeBottom.setControl(m_BottomVoltage);
  
    m_TopVoltage.withVelocity(goalSpeedTop);
    OuttakeLT.setControl(m_TopVoltage);
    OuttakeRB.setControl(m_TopVoltage);
    OuttakeRT.setControl(m_TopVoltage);

    m_BackVoltage.withVelocity(goalSpeedBack);
    OuttakeLB.setControl(m_BackVoltage);

    if(goalSpeedBottom == 0){
      OuttakeBottom.setVoltage(0);
    }
    if(goalSpeedTop == 0){
      OuttakeLT.setVoltage(0);
      OuttakeRB.setVoltage(0);
      OuttakeRT.setVoltage(0);
    }
    if(goalSpeedBack == 0){
      OuttakeLB.setVoltage(0);
    }
    
    SmartDashboard.putNumber("Outtake LT Speed", OuttakeLT.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Outtake RB Speed", OuttakeRB.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Outtake RT Speed", OuttakeRT.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Outtake Top Goal", goalSpeedTop);

    SmartDashboard.putNumber("Outtake Bottom Speed", OuttakeBottom.getVelocity().getValueAsDouble()); 
    SmartDashboard.putNumber("Outtake Bottom Goal", goalSpeedBottom);  

    SmartDashboard.putNumber("Outake LB Speed", OuttakeLB.getVelocity().getValueAsDouble()); 
    SmartDashboard.putNumber("Outtake Back Goal", goalSpeedBack);  
  }

  // GENERAL METHODS
  public void start() {
    this.goalSpeedBottom = 0;
    this.goalSpeedTop = 0;
    this.goalSpeedBack = 0;
  }
  
  public void stopLauncher(){
    this.goalSpeedBottom = 0;
    this.goalSpeedTop = 0;
    this.goalSpeedBack = 0;
  } 

  public Command startCommand(){
    Command result = runOnce(this::start);
    return result;
  }

  public Command stopCommand(){
    Command result = runOnce(this::stopLauncher);
    return result;
  }

  // TOP ROLLER METHODS
  public void incrementSpeedTop(double rpsChange) {
    this.goalSpeedTop += rpsChange;
  }

  public void setSpeedTop(double rps) {
    this.goalSpeedTop = rps;
  }

  public Command incrementSpeedTopCommand(double rpsChange){
    Command result = runOnce(()-> incrementSpeedTop(rpsChange));
    return result;
  } 

  public Command setSpeedTopCommand(double rps){
    Command result = runOnce(()-> setSpeedTop(rps));
    return result;
  } 

  // BOTTOM ROLLER METHODS
  public void setSpeedBottom(double rps) {
    this.goalSpeedBottom = rps;
  }

  public void incrementSpeedBottom(double rpsChange) {
    this.goalSpeedBottom += rpsChange;
  }

  public Command incrementSpeedBottomCommand(double rpsChange){
    Command result = runOnce(()-> incrementSpeedBottom(rpsChange));
    return result;
  } 
  
  public Command setSpeedBottomCommand(double rps){
    Command result = runOnce(()-> setSpeedBottom(rps));
    return result;
  } 

  // BACK ROLLER METHODS
  public void setSpeedBack(double rps) {
    this.goalSpeedBack = rps;
  }

  public void incrementSpeedBack(double rpsChange) {
    this.goalSpeedBack += rpsChange;
  }

  public Command incrementSpeedBackCommand(double rpsChange){
    Command result = runOnce(()-> incrementSpeedBack(rpsChange));
    return result;
  } 
  
  public Command setSpeedBackCommand(double rps){
    Command result = runOnce(()-> setSpeedBack(rps));
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