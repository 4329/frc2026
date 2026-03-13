package frc.robot.subsystems;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.model.IntakeLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public class SpinDexterSubsystem extends SubsystemBase implements LoggedSubsystem {
    private final TalonFX spinnerMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final IntakeLogAutoLogged rotateLog = new IntakeLogAutoLogged();

    private static final double TOLERANCE = 0.5;
    private double targetPosition = 0.0;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public SpinDexterSubsystem() {
        spinnerMotor = new TalonFX(15);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(1.0);
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

        // ✅ Signals initialized here
        positionSignal      = spinnerMotor.getPosition();
        velocitySignal      = spinnerMotor.getVelocity();
        appliedVoltsSignal  = spinnerMotor.getMotorVoltage();
        supplyCurrentSignal = spinnerMotor.getSupplyCurrent();
        torqueCurrentSignal = spinnerMotor.getTorqueCurrent();
        tempSignal          = spinnerMotor.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, tempSignal
        );
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

    public boolean atPosition(double target, double tolerance) {
        return Math.abs(positionSignal.getValueAsDouble() - target) <= tolerance;
    }

    public void spinVoltage(double voltage) {
        spinnerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        targetPosition = rotations; // ✅ track for logging
        spinnerMotor.setControl(positionRequest.withPosition(rotations));
    }

    public void setVelocity(double rotationsPerSecond) {
        spinnerMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond)); // ✅ fixed - was using voltageRequest
    }

    public double getPosition() {
        return spinnerMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return spinnerMotor.getVelocity().getValueAsDouble(); // ✅ fixed - was throwing UnsupportedOperationException
    }

    public void stop() {
        spinnerMotor.stopMotor();
    }

}