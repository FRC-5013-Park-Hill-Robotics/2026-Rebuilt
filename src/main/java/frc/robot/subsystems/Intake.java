package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.trobot5013lib.AverageOverTime;

public class Intake extends SubsystemBase {
    private TalonFX intakeLeftMotor = new TalonFX(CANConstants.INTAKE_L_ID, CANConstants.CANBUS_AUX);
    private TalonFX intakeRightMotor = new TalonFX(CANConstants.INTAKE_R_ID, CANConstants.CANBUS_AUX);
    private TalonFX intakeMoverMotor = new TalonFX(CANConstants.INTAKE_MOVER_ID, CANConstants.CANBUS_AUX);

    private VelocityVoltage m_IntakeVelocityVoltage = new VelocityVoltage(0);
    private double target = 0;
    private SlewRateLimiter m_intakeWheelLimiter = new SlewRateLimiter(400);
    private SlewRateLimiter m_moverlimiter = new SlewRateLimiter(5);

    private VoltageOut m_MoverVelocityVoltage = new VoltageOut(0);
    private final Timer m_moverTimer = new Timer();
    private double m_HoldPosition = 0;

    private AverageOverTime MoveOutAoT = new AverageOverTime(0.3);

    public enum IntakeState{
        In,
        TowardsIn,
        Aggitate,
        TowardsOut,
        Out
    }
    private IntakeState m_intakePosition = IntakeState.In;

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
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        intakeRightMotor.getConfigurator().apply(config);
        intakeRightMotor.set(0);

        TalonFXConfiguration config2 = new TalonFXConfiguration();
        config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config2.Slot0.kP = IntakeConstants.RollerGains.kP;
        config2.Slot0.kI = IntakeConstants.RollerGains.kI;
        config2.Slot0.kD = IntakeConstants.RollerGains.kD;
        config2.Slot0.kS = IntakeConstants.RollerGains.kS;
        config2.Slot0.kV = IntakeConstants.RollerGains.kV;
        config2.Slot0.kA = IntakeConstants.RollerGains.kA;
        config2.CurrentLimits.StatorCurrentLimit = 80;
        config2.CurrentLimits.SupplyCurrentLimit = 60;
        intakeRightMotor.getConfigurator().apply(config2);
        intakeRightMotor.setControl(new Follower(CANConstants.INTAKE_L_ID, MotorAlignmentValue.Opposed));
        intakeRightMotor.set(0);

        TalonFXConfiguration config3 = new TalonFXConfiguration();
        config3.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config3.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config3.Slot0.kP = IntakeConstants.IntakeMoverGains.kP;
        config3.Slot0.kI = IntakeConstants.IntakeMoverGains.kI;
        config3.Slot0.kD = IntakeConstants.IntakeMoverGains.kD;
        config3.Slot0.kS = IntakeConstants.IntakeMoverGains.kS;
        config3.Slot0.kV = IntakeConstants.IntakeMoverGains.kV;
        config3.Slot0.kA = IntakeConstants.IntakeMoverGains.kA;
        config3.CurrentLimits.StatorCurrentLimit = 60;
        config3.CurrentLimits.SupplyCurrentLimit = 40;
        intakeMoverMotor.getConfigurator().apply(config3);
        intakeMoverMotor.set(0);

        m_IntakeVelocityVoltage.withSlot(0);
        m_MoverVelocityVoltage.withOutput(0);
        m_moverTimer.start();

        m_HoldPosition = 0;
    }

    @Override
    public void periodic() {
        m_IntakeVelocityVoltage.withVelocity(m_intakeWheelLimiter.calculate(target));
        intakeLeftMotor.setControl(m_IntakeVelocityVoltage);
        //intakeRightMotor.setControl(m_IntakeVelocityVoltage); Follower

        double intakePos = intakeMoverMotor.getPosition().getValueAsDouble();
        double intakeTime = intakeMoverMotor.getPosition().getTimestamp().getTime();

        String state = "Null";

        MoveOutAoT.addMessurement(intakePos, intakeTime);
        // !hasElapsed so it only runs for moverIn/OutTime seconds
        if(m_intakePosition == IntakeState.TowardsOut){
            state = "Towards Out";
            m_MoverVelocityVoltage.withOutput(m_moverlimiter.calculate(IntakeConstants.moveOutVolt));
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
            
            if(m_moverTimer.hasElapsed(IntakeConstants.moveTime) /*&& Math.abs(MoveOutAoT.getAverage(intakeTime) - intakePos) < IntakeConstants.noMovingTollerance*/){
                m_intakePosition = IntakeState.Out;
            }
        }
        if(m_intakePosition == IntakeState.TowardsIn){
            state = "Towards In";
            m_MoverVelocityVoltage.withOutput(m_moverlimiter.calculate(IntakeConstants.moveInVolt));
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
            
            if(m_moverTimer.hasElapsed(IntakeConstants.moveTime) /*&& Math.abs(MoveOutAoT.getAverage(intakeTime) - intakePos) < IntakeConstants.noMovingTollerance*/){
                m_intakePosition = IntakeState.In;
            }
        }
        if(m_intakePosition == IntakeState.Aggitate){
            state = "Aggitating";
            m_MoverVelocityVoltage.withOutput(m_moverlimiter.calculate(IntakeConstants.moveInVolt));
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
            
            if(m_moverTimer.hasElapsed(IntakeConstants.moverAggitateTime)){
                m_moverTimer.reset();
                moveIntakeOut();
            }
        }
        if(m_intakePosition == IntakeState.In){
            state = "In";
            m_MoverVelocityVoltage.withOutput(0);
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
        }
        if(m_intakePosition == IntakeState.Out){
            state = "Out";
            m_MoverVelocityVoltage.withOutput(0);
            intakeMoverMotor.setControl(m_MoverVelocityVoltage);
        }

        // m_MoverVelocityVoltage.withOutput(m_moverlimiter.calculate(intakeMoverMotor.getPosition().getValueAsDouble()-m_HoldPosition));
        // intakeMoverMotor.setControl(m_MoverVelocityVoltage);

        SmartDashboard.putString("Intake: Intake State", state);

        SmartDashboard.putNumber("Intake: Mover Current AoT", MoveOutAoT.getAverage(intakeMoverMotor.getPosition().getTimestamp().getTime()));
        SmartDashboard.putNumber("Intake: Mover Hold Position", m_HoldPosition);
        SmartDashboard.putNumber("Intake: Mover Position", intakeMoverMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake: Mover Velocity", m_MoverVelocityVoltage.Output);
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
        m_moverTimer.reset();
        m_intakePosition = IntakeState.TowardsIn;
    }

    public void moveIntakeOut(){
        m_moverTimer.reset();
        m_intakePosition = IntakeState.TowardsOut;
    }

    public void aggitateIntake(){
        m_moverTimer.reset();
        m_intakePosition = IntakeState.Aggitate;
    }

    public void toggleIntakePosition(){
        if(m_intakePosition == IntakeState.In /*|| m_intakePosition == IntakeState.TowardsIn*/){
            moveIntakeOut();
        }
        if(m_intakePosition == IntakeState.Out /*|| m_intakePosition == IntakeState.TowardsOut*/){
            moveIntakeIn();
        }
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

    public Command aggitateIntakeC(){
        Command result = runOnce(()-> aggitateIntake());
        return result;
    } 

    public Command toggleIntakePositionC(){
        Command result = runOnce(()-> toggleIntakePosition());
        return result;
    } 
}
