package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private TalonFX intakeRollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_ID, CANConstants.CANBUS_AUX);

    private VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0);

    private double target = 0;
    private SlewRateLimiter limiter = new SlewRateLimiter(400);

    public Intake() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.kP = IntakeConstants.RollerGains.kP;
        config.Slot0.kI = IntakeConstants.RollerGains.kI;
        config.Slot0.kD = IntakeConstants.RollerGains.kD;
        config.Slot0.kS = IntakeConstants.RollerGains.kS;
        config.Slot0.kV = IntakeConstants.RollerGains.kV;
        config.Slot0.kA = IntakeConstants.RollerGains.kA;
        config.CurrentLimits.StatorCurrentLimit = 40;
        intakeRollerMotor.getConfigurator().apply(config);
        intakeRollerMotor.set(0);
        m_VelocityVoltage.withSlot(0);
    }

    @Override
    public void periodic() {
        m_VelocityVoltage.withVelocity(target);
        intakeRollerMotor.setControl(m_VelocityVoltage);
    }

    public void stop() {
        target = 0;
    }

    public void setTarget(double targetSpeed){
        target = targetSpeed;
    }

    public void setTarget(double targetSpeed, double elseSpeed, BooleanSupplier suppy){
        if (suppy.getAsBoolean()) {
            target = elseSpeed;
        }
        else{
            target = targetSpeed;
        }
    }

    public void incrementRollers(double amount){
        target += amount;
    }
    
    public Command stopC(){
        Command result = runOnce(this::stop);
        return result;
    }


    public Command setTargetC(double targetSpeed){
        Command result = runOnce(() -> setTarget(targetSpeed));
        return result;
    }

    public Command incrementRollersC(double rotationChange){
        Command result = runOnce(()-> incrementRollers(rotationChange));
        return result;
    } 
}
