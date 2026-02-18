package frc.robot.subsystems.TurretSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.TurretRotateLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RotateSubsystem extends SubsystemBase implements LoggedSubsystem {

    private final TalonFX turretMotor;
    private final VoltageOut voltageRequest             = new VoltageOut(0);
    private final PositionVoltage positionRequest       = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest       = new VelocityVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final TurretRotateLogAutoLogged rotateLog   = new TurretRotateLogAutoLogged();

    private static final double MIN_POSITION = -1.2;
    private static final double MAX_POSITION = 1.2;
    private static final double TOLERANCE = 0.01;

    private double targetPosition = 0.0;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public RotateSubsystem() {
        turretMotor = new TalonFX(41);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(1);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.02);
        Slot0Configs.withKV(0.12);

        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(3);
        motionMagicConfigs.withMotionMagicAcceleration(10);
        motionMagicConfigs.withMotionMagicJerk(50);

        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);

        var softLimitsConfigs = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        softLimitsConfigs.withForwardSoftLimitThreshold(1.2);
        softLimitsConfigs.withForwardSoftLimitEnable(true);
        softLimitsConfigs.withReverseSoftLimitThreshold(-1.2);
        softLimitsConfigs.withReverseSoftLimitEnable(true);

        turretMotor.getConfigurator().apply(motorOutputConfigs);
        turretMotor.getConfigurator().apply(Slot0Configs);
        turretMotor.getConfigurator().apply(motionMagicConfigs);
        turretMotor.getConfigurator().apply(feedbackConfigs);
        turretMotor.getConfigurator().apply(softLimitsConfigs);
        turretMotor.setPosition(0);

        positionSignal      = turretMotor.getPosition();
        velocitySignal      = turretMotor.getVelocity();
        appliedVoltsSignal  = turretMotor.getMotorVoltage();
        supplyCurrentSignal = turretMotor.getSupplyCurrent();
        torqueCurrentSignal = turretMotor.getTorqueCurrent();
        tempSignal          = turretMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, tempSignal
        );
        turretMotor.optimizeBusUtilization();

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    @Override
    public LoggableInputs log() {
        rotateLog.motorConnected = BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, tempSignal
        ).isOK();

        rotateLog.positionRotations       = positionSignal.getValueAsDouble();
        rotateLog.velocityRotationsPerSec = velocitySignal.getValueAsDouble();
        rotateLog.appliedVolts            = appliedVoltsSignal.getValueAsDouble();
        rotateLog.supplyCurrentAmps       = supplyCurrentSignal.getValueAsDouble();
        rotateLog.torqueCurrentAmps       = torqueCurrentSignal.getValueAsDouble();
        rotateLog.tempCelsius             = tempSignal.getValueAsDouble();
        rotateLog.targetPosition          = targetPosition;
        rotateLog.positionError           = targetPosition - positionSignal.getValueAsDouble();
        rotateLog.atTarget                = atPosition(targetPosition, TOLERANCE);

        return rotateLog;
    }

    @Override
    public String getNameLog() {
        return "Turret/Rotate";
    }

    public void spinVoltage(double voltage) {
        turretMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        turretMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    public void setVelocity(double rotationsPerSecond) {
        turretMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    public void setPositionWithVelocity(double rotations) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        turretMotor.setControl(motionMagicRequest.withPosition(targetPosition));
    }

    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }

    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    public boolean atPosition(double targetPosition, double tolerance) {
        return Math.abs(getPosition() - targetPosition) < tolerance;
    }

    public void stop() {
        turretMotor.stopMotor();
    }
}