package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FakeTurretMotorSubsystem extends SubsystemBase {
    private final TalonFX fakeTurretMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public FakeTurretMotorSubsystem() {
        fakeTurretMotor = new TalonFX(13);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.withKP(2.0);
        slot0Configs.withKI(0.0);
        slot0Configs.withKD(0.0);

        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);

        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(5.0);
        motionMagicConfigs.withMotionMagicAcceleration(10.0);
        motionMagicConfigs.withMotionMagicJerk(100.0);

        fakeTurretMotor.getConfigurator().apply(motorOutputConfigs);
        fakeTurretMotor.getConfigurator().apply(slot0Configs);
        fakeTurretMotor.getConfigurator().apply(feedbackConfigs);
        fakeTurretMotor.getConfigurator().apply(motionMagicConfigs);

        fakeTurretMotor.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    public void spinVoltage(double voltage) {
        fakeTurretMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        fakeTurretMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    public double getPosition() {
        return fakeTurretMotor.getPosition().getValueAsDouble();
    }

    public void stop() {
        fakeTurretMotor.stopMotor();
    }
}