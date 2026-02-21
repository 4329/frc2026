package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MotorLogger;

public class SpinDexterSubsystem extends SubsystemBase {
    private final TalonFX spinnerMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public SpinDexterSubsystem() {
        spinnerMotor = new TalonFX(44);

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
        
        spinnerMotor.getConfigurator().apply(motorOutputConfigs);
        spinnerMotor.getConfigurator().apply(Slot0Configs);
        spinnerMotor.getConfigurator().apply(feedbackConfigs);

        spinnerMotor.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    public void spinVoltage(double voltage) {
        spinnerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        spinnerMotor.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return spinnerMotor.getPosition().getValueAsDouble();
    }

    public void stop() {
        spinnerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        MotorLogger.logTalonFX(spinnerMotor, "SpinDexter/SpinnerMotor");
    }
}