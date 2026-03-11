package frc.robot.subsystems.TurretSubsystem;

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
import frc.robot.model.ShooterLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterSubsystem extends SubsystemBase implements LoggedSubsystem {

    private final TalonFX shooterMotor;
    private final VoltageOut voltageRequest       = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final ShooterLogAutoLogged shooterLog = new ShooterLogAutoLogged();

    private static final double TOLERANCE = 2.0;
    private double targetVelocity = 0.0;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public ShooterSubsystem() {
        shooterMotor = new TalonFX(19);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(3.0);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.0);

        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);


        shooterMotor.getConfigurator().apply(motorOutputConfigs);
        shooterMotor.getConfigurator().apply(Slot0Configs);
        shooterMotor.getConfigurator().apply(feedbackConfigs);
        shooterMotor.setPosition(0);

        positionSignal      = shooterMotor.getPosition();
        velocitySignal      = shooterMotor.getVelocity();
        appliedVoltsSignal  = shooterMotor.getMotorVoltage();
        supplyCurrentSignal = shooterMotor.getSupplyCurrent();
        torqueCurrentSignal = shooterMotor.getTorqueCurrent();
        tempSignal          = shooterMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, tempSignal
        );
        shooterMotor.optimizeBusUtilization();

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    @Override
    public LoggableInputs log() {
        shooterLog.motorConnected = BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, tempSignal
        ).isOK();

        shooterLog.positionRotations       = positionSignal.getValueAsDouble();
        shooterLog.velocityRotationsPerSec = velocitySignal.getValueAsDouble();
        shooterLog.appliedVolts            = appliedVoltsSignal.getValueAsDouble();
        shooterLog.supplyCurrentAmps       = supplyCurrentSignal.getValueAsDouble();
        shooterLog.torqueCurrentAmps       = torqueCurrentSignal.getValueAsDouble();
        shooterLog.tempCelsius             = tempSignal.getValueAsDouble();
        shooterLog.targetVelocity          = targetVelocity;
        shooterLog.velocityError           = targetVelocity - velocitySignal.getValueAsDouble();
        shooterLog.atSpeed                 = Math.abs(getVelocity() - targetVelocity) < TOLERANCE;

        return shooterLog;
    }

    @Override
    public String getNameLog() {
        return "Turret/Shooter";
    }

    public void spinVoltage(double voltage) {
        shooterMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        shooterMotor.setControl(positionRequest.withPosition(rotations));
    }

    public void setVelocity(double rotationsPerSecond) {
        targetVelocity = rotationsPerSecond;
        shooterMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }

    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    public void stop() {
        targetVelocity = 0.0;
        shooterMotor.stopMotor();
    }
}