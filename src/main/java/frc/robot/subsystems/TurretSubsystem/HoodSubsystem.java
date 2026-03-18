package frc.robot.subsystems.TurretSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.HoodLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HoodSubsystem extends SubsystemBase implements LoggedSubsystem {

    public final TalonFX hoodMotor;
    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
    private final HoodLogAutoLogged hoodLog = new HoodLogAutoLogged();

    private static final double MIN_POSITION = 0.1;
    private static final double MAX_POSITION = 6.3;
    private static final double TOLERANCE = 0.01;

    private double targetPosition = MIN_POSITION;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public HoodSubsystem() {
        hoodMotor = new TalonFX(18);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(12.0);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.001);
        Slot0Configs.withKV(1);

        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(0.1);
        motionMagicConfigs.withMotionMagicAcceleration(0.05);
        motionMagicConfigs.withMotionMagicJerk(0.5);

        var softLimitConfigs = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        softLimitConfigs.withForwardSoftLimitThreshold(6.2);
        softLimitConfigs.withForwardSoftLimitEnable(true);
        softLimitConfigs.withReverseSoftLimitThreshold(0.0);
        softLimitConfigs.withReverseSoftLimitEnable(true);

        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);

        hoodMotor.getConfigurator().apply(motorOutputConfigs);
        hoodMotor.getConfigurator().apply(Slot0Configs);
        hoodMotor.getConfigurator().apply(motionMagicConfigs);
        hoodMotor.getConfigurator().apply(feedbackConfigs);
        hoodMotor.setPosition(0.0);

        positionSignal      = hoodMotor.getPosition();
        velocitySignal      = hoodMotor.getVelocity();
        appliedVoltsSignal  = hoodMotor.getMotorVoltage();
        supplyCurrentSignal = hoodMotor.getSupplyCurrent();
        torqueCurrentSignal = hoodMotor.getTorqueCurrent();
        tempSignal          = hoodMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, tempSignal
        );
        hoodMotor.optimizeBusUtilization();

        // setDefaultCommand(Commands.run(() -> holdPosition(), this));
        setDefaultCommand(Commands.run(() -> stop(), this));

    }

    @Override
    public LoggableInputs log() {
        hoodLog.motorConnected = BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, tempSignal
        ).isOK();

        hoodLog.positionRotations       = positionSignal.getValueAsDouble();
        hoodLog.velocityRotationsPerSec = velocitySignal.getValueAsDouble();
        hoodLog.appliedVolts            = appliedVoltsSignal.getValueAsDouble();
        hoodLog.supplyCurrentAmps       = supplyCurrentSignal.getValueAsDouble();
        hoodLog.torqueCurrentAmps       = torqueCurrentSignal.getValueAsDouble();
        hoodLog.tempCelsius             = tempSignal.getValueAsDouble();
        hoodLog.targetPosition          = targetPosition;
        hoodLog.positionError           = targetPosition - positionSignal.getValueAsDouble();
        hoodLog.atTarget                = atPosition(targetPosition, TOLERANCE);

        return hoodLog;
    }

    @Override
    public String getNameLog() {
        return "Turret/Hood";
    }

    public void setPosition(double rotations) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        hoodMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    public void holdPosition() {
        setPosition(getPosition());
    }

    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }

    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    public double getSupplyCurrent() {
        supplyCurrentSignal.refresh();
        return supplyCurrentSignal.getValueAsDouble();
    }

    public boolean atPosition(double targetPosition, double tolerance) {
        return Math.abs(getPosition() - targetPosition) < tolerance;
    }

    public void stop() {
        hoodMotor.stopMotor();
    }
}