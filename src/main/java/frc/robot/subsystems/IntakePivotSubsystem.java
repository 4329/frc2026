package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakePivotSubsystem extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private static final double MIN_POSITION = 0;
    private static final double MAX_POSITION = 4;
    private double targetPosition = MIN_POSITION;

    public IntakePivotSubsystem() {
        pivotMotor =  new TalonFX(13);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(0.5);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.0);
        Slot0Configs.withKV(0.12);
        
        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);

        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(0.05);
        motionMagicConfigs.withMotionMagicAcceleration(0.01);
        motionMagicConfigs.withMotionMagicJerk(0.1);

        var softLimitConfigs = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        softLimitConfigs.withForwardSoftLimitThreshold(4);
        softLimitConfigs.withForwardSoftLimitEnable(true);
        softLimitConfigs.withReverseSoftLimitThreshold(0);
        softLimitConfigs.withReverseSoftLimitEnable(true);

        var currentLimitConfigs = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimitConfigs.withStatorCurrentLimit(40);
        currentLimitConfigs.withStatorCurrentLimitEnable(true);
        currentLimitConfigs.withSupplyCurrentLimit(40);
        currentLimitConfigs.withSupplyCurrentLimitEnable(true);
        
        pivotMotor.getConfigurator().apply(motorOutputConfigs);
        pivotMotor.getConfigurator().apply(Slot0Configs);
        pivotMotor.getConfigurator().apply(feedbackConfigs);
        pivotMotor.getConfigurator().apply(motionMagicConfigs);
        pivotMotor.getConfigurator().apply(softLimitConfigs);
        pivotMotor.getConfigurator().apply(currentLimitConfigs);

        pivotMotor.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    public void spinVoltage(double voltage) {
        pivotMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        pivotMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public void stop() {
        pivotMotor.stopMotor();
    }
}
