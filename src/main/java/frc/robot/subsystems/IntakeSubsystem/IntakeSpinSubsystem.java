package frc.robot.subsystems.IntakeSubsystem;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.IntakeLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public class IntakeSpinSubsystem extends SubsystemBase implements LoggedSubsystem{
    private final TalonFX spinIntakeMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final IntakeLogAutoLogged rotateLog   = new IntakeLogAutoLogged();


    
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;



    public IntakeSpinSubsystem() {
        spinIntakeMotor =  new TalonFX(14);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Coast);
        motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(10.0);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.0);

        
        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);

        var currentLimitConfigs = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimitConfigs.withStatorCurrentLimit(40);
        currentLimitConfigs.withStatorCurrentLimitEnable(true);
        currentLimitConfigs.withSupplyCurrentLimit(40);
        currentLimitConfigs.withSupplyCurrentLimitEnable(true);
        
        spinIntakeMotor.getConfigurator().apply(motorOutputConfigs);
        spinIntakeMotor.getConfigurator().apply(Slot0Configs);
        spinIntakeMotor.getConfigurator().apply(feedbackConfigs);
        spinIntakeMotor.getConfigurator().apply(currentLimitConfigs);

        spinIntakeMotor.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
        
        positionSignal      = spinIntakeMotor.getPosition();
        velocitySignal      = spinIntakeMotor.getVelocity();
        appliedVoltsSignal  = spinIntakeMotor.getMotorVoltage();
        supplyCurrentSignal = spinIntakeMotor.getSupplyCurrent();
        torqueCurrentSignal = spinIntakeMotor.getTorqueCurrent();
        tempSignal          = spinIntakeMotor.getDeviceTemp();
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
        
        return rotateLog;
    }

    public void spinVoltage(double voltage) {
        spinIntakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        spinIntakeMotor.setControl(positionRequest.withPosition(rotations));
    }

    public void setVelocity(double rotationsPerSecond) {
        spinIntakeMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    public double getPosition() {
        return spinIntakeMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return spinIntakeMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        spinIntakeMotor.stopMotor();
    }
}