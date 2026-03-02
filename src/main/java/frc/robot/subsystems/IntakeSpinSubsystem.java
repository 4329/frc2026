package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSpinSubsystem extends SubsystemBase {
    private final TalonFX spinMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public IntakeSpinSubsystem() {
        spinMotor =  new TalonFX(13);

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
        
        spinMotor.getConfigurator().apply(motorOutputConfigs);
        spinMotor.getConfigurator().apply(Slot0Configs);
        spinMotor.getConfigurator().apply(feedbackConfigs);

        spinMotor.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    public void spinVoltage(double voltage) {
        spinMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        spinMotor.setControl(positionRequest.withPosition(rotations));
    }

    public void setVelocity(double rotationsPerSecond) {
        spinMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    public double getPosition() {
        return spinMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return spinMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        spinMotor.stopMotor();
    }
}
