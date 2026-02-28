package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.constants.IntakeConstants.IntakeMoverGains;

public class Intake extends SubsystemBase {
    private TalonFX intakeRollerMotor = new TalonFX(CANConstants.INTAKE_MAIN_ID, CANConstants.CANBUS_AUX);
    private TalonFX intakeMoverMotor = new TalonFX(CANConstants.INTAKE_RIGHT_ID, CANConstants.CANBUS_AUX);

    private VelocityVoltage m_IntakeVelocityVoltage = new VelocityVoltage(0);
    private double target = 0;
    private SlewRateLimiter m_intakelimiter = new SlewRateLimiter(400);
    
    private VoltageOut m_MoverVelocityVoltage = new VoltageOut(0);
    private int m_moverDirection = 1;
    private SlewRateLimiter m_moverlimiter = new SlewRateLimiter(5);
    private final Timer m_moverTimer = new Timer();

    public Intake() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

        TalonFXConfiguration config2 = new TalonFXConfiguration();
        config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config2.Slot0.kP = IntakeConstants.IntakeMoverGains.kP;
        config2.Slot0.kI = IntakeConstants.IntakeMoverGains.kI;
        config2.Slot0.kD = IntakeConstants.IntakeMoverGains.kD;
        config2.Slot0.kS = IntakeConstants.IntakeMoverGains.kS;
        config2.Slot0.kV = IntakeConstants.IntakeMoverGains.kV;
        config2.Slot0.kA = IntakeConstants.IntakeMoverGains.kA;
        config2.CurrentLimits.StatorCurrentLimit = 40;
        intakeMoverMotor.getConfigurator().apply(config2);
        intakeMoverMotor.set(0);

        m_IntakeVelocityVoltage.withSlot(0);
        m_MoverVelocityVoltage.withOutput(0);
        m_moverTimer.start();
    }

    @Override
    public void periodic() {
        m_IntakeVelocityVoltage.withVelocity(m_intakelimiter.calculate(target));
        intakeRollerMotor.setControl(m_IntakeVelocityVoltage);

        // !hasElapsed so it only runs for moverIn/OutTime seconds
        if(m_moverDirection == -1 && !m_moverTimer.hasElapsed(IntakeConstants.moverInTime)){
            m_MoverVelocityVoltage.withOutput(IntakeConstants.moveInVolt);
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
        }
        else if(m_moverDirection == 1 && !m_moverTimer.hasElapsed(IntakeConstants.moverOutTime)){
            m_MoverVelocityVoltage.withOutput(IntakeConstants.moveOutVolt);
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
        }
        else{
            m_MoverVelocityVoltage.withOutput(0);
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
        }
    }

    // General Functions
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

    public void moveIntakeIn(){
        m_moverDirection = -1;
        m_moverTimer.reset();
    }

    public void moveIntakeOut(){
        m_moverDirection = 1;
        m_moverTimer.reset();
    }
    
    // Commands
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

    public Command moveIntakeInC(){
        Command result = runOnce(() -> moveIntakeIn());
        return result;
    }

    public Command moveIntakeOutC(){
        Command result = runOnce(()-> moveIntakeOut());
        return result;
    } 
}
