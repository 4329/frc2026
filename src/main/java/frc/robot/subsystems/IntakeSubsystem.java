package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MotorLogger;

public class IntakeSubsystem extends SubsystemBase {
    // Hardware - Kraken X60 on CAN ID 43
    private final TalonFX m_kraken = new TalonFX(43);
    
    // Control Request for Position
    private final PositionVoltage m_positionRequest = new PositionVoltage(0);

    // Math: Rotations = Degrees / 360
    private final double kGearRatio = 1.0; 
    private final double kPos0 = (0.0 / 360.0) * kGearRatio;
    private final double kPos215 = (215.0 / 360.0) * kGearRatio;

    public IntakeSubsystem() {
        // 1. Configure PID
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 12.0; 
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.1; 
        m_kraken.getConfigurator().apply(slot0Configs);

        // 2. Configure Soft Limits
        var softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitThreshold = kPos215 + 0.05; 
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = kPos0 - 0.05;
        softLimits.ReverseSoftLimitEnable = true;
        m_kraken.getConfigurator().apply(softLimits);

        // 3. Set to Brake Mode
        m_kraken.setNeutralMode(NeutralModeValue.Brake);
    }

    /** Moves the motor to 215 degrees and locks it there */
    public void goTo215() {
        m_kraken.setControl(m_positionRequest.withPosition(kPos215));
    }

    /** Moves the motor to 0 degrees and locks it there */
    public void goTo0() {
        m_kraken.setControl(m_positionRequest.withPosition(kPos0));
    }

    /** Moves to a specific position in rotations */
    public void setPosition(double targetPosition) {
        m_kraken.setControl(m_positionRequest.withPosition(targetPosition));
    }

    /** Gets current position in rotations */
    public double getPosition() {
        return m_kraken.getPosition().getValueAsDouble();
    }

    /** Spins the motor at a specific voltage (open loop) */
    public void spinVoltage(double voltage) {
        m_kraken.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage));
    }

    /** Completely stops motor output */
    public void stop() {
        m_kraken.setControl(new com.ctre.phoenix6.controls.NeutralOut());
    }

    /** Get the target position for 0 degrees */
    public double getPos0() {
        return kPos0;
    }

    /** Get the target position for 215 degrees */
    public double getPos215() {
        return kPos215;
    }

    @Override
    public void periodic() {
        MotorLogger.logTalonFX(m_kraken, "Intake/Kraken");
    }
}