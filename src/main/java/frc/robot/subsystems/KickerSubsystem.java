package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import frc.robot.utilities.MotorLogger;

// not in use currently, keeping as backup

public class KickerSubsystem extends SubsystemBase {
    private final TalonFX spinnerMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0); // ✅ fixed

    private static final double TOLERANCE = 0.5;
    private double targetPosition = 0.0;
    private final int m_motorID;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public KickerSubsystem(int motorID) {
        m_motorID = motorID;
        spinnerMotor = new TalonFX(motorID);

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
        spinnerMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond)); // ✅ fixed
    }

    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }

    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    public void stop() {
        spinnerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        MotorLogger.logTalonFX(spinnerMotor, "Kicker/Motor_" + m_motorID);
    }
}