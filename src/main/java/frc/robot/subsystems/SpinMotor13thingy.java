package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpinMotor13thingy extends SubsystemBase {

    private final TalonFX m_kraken = new TalonFX(13);
    
    private final VoltageOut m_voltageRequest = new VoltageOut(0);
    private final NeutralOut m_neutralRequest = new NeutralOut();

    public SpinMotor13thingy() {
        m_kraken.setNeutralMode(NeutralModeValue.Brake);
    }

    public void spinVoltage(double voltage) {
        m_kraken.setControl(m_voltageRequest.withOutput(voltage));
    }

    public void stop() {
        m_kraken.setControl(m_neutralRequest);
    }
}
