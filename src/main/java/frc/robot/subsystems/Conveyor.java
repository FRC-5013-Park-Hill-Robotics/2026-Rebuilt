// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  
  private TalonFX ConveyorMotor = new TalonFX(CANConstants.INTAKE_LEFT_ID, CANConstants.CANBUS_AUX);
  
  private VelocityVoltage m_Voltage = new VelocityVoltage(0);
  private double goalSpeed = 0;

  /** Creates a new Conveyor. */
  public Conveyor() {
    TalonFXConfiguration Config = new TalonFXConfiguration();
    Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    Config.Slot0.kP = ConveyorConstants.RollerGains.kP; 
    Config.Slot0.kI = ConveyorConstants.RollerGains.kI;
    Config.Slot0.kD = ConveyorConstants.RollerGains.kD;
    Config.Slot0.kS = ConveyorConstants.RollerGains.kS;
    Config.Slot0.kV = ConveyorConstants.RollerGains.kV;
    Config.Slot0.kA = ConveyorConstants.RollerGains.kA;
    ConveyorMotor.set(0);
    ConveyorMotor.getConfigurator().apply(Config);
    m_Voltage.withSlot(0);
  }

  @Override
  public void periodic() {
    m_Voltage.withVelocity(goalSpeed);
    ConveyorMotor.setControl(m_Voltage);
  }

  public void stop() {
    goalSpeed = 0;
  }

  public void setTarget(double targetSpeed){
    goalSpeed = targetSpeed;
  }

  public void setTarget(double targetSpeed, double elseSpeed, BooleanSupplier suppy){
    if (suppy.getAsBoolean()) {
      goalSpeed = elseSpeed;
    }
    else{
      goalSpeed = targetSpeed;
    }
  }

  public void increment(double amount){
    goalSpeed += amount;
  }
  
  public Command stopC(){
    Command result = runOnce(this::stop);
    return result;
  }


  public Command setTargetC(double targetSpeed){
    Command result = runOnce(() -> setTarget(targetSpeed));
    return result;
  }

  public Command incrementC(double rotationChange){
    Command result = runOnce(()-> increment(rotationChange));
    return result;
  } 
}
