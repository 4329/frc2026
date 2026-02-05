package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenMotorSubsystem extends SubsystemBase {
    private final TalonFX motorYes;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public KrakenMotorSubsystem() {
        motorYes = new TalonFX(43);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(10.0);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.0);

        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);

        motorYes.getConfigurator().apply(motorOutputConfigs);
        motorYes.getConfigurator().apply(Slot0Configs);
        motorYes.getConfigurator().apply(feedbackConfigs);

        motorYes.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    public void spinVoltage(double voltage) {
        motorYes.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        motorYes.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return motorYes.getPosition().getValueAsDouble();
    
    }

    public void stop() {
        motorYes.stopMotor();
    }

    
}
