package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.model.IntakePivotLogAutoLogged;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.model.IntakePivotLog;


public class IntakePivotSubsystem extends SubsystemBase implements LoggedSubsystem{
    public final TalonFX pivotMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    private final IntakePivotLogAutoLogged inputs = new IntakePivotLogAutoLogged();
    public double motorVoltage = 0.0;
    private static final double AT_TARGET_TOLERANCE = 0.1;
    
    private static final double MIN_POSITION = 0;
    private static final double MAX_POSITION = 6.2;
    private double targetPosition = MIN_POSITION;

    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Voltage>     appliedVoltsSignal;
    private final StatusSignal<Temperature> tempSignal;

    public IntakePivotSubsystem() {
        pivotMotor =  new TalonFX(13);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(1.0);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.0);
        Slot0Configs.withKV(0);
        Slot0Configs.withKS(0);
        // Slot0Configs.withGravityType(GravityTypeValue.Arm_Cosine);
        
        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);

        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(5.0);
        motionMagicConfigs.withMotionMagicAcceleration(2.5);
        motionMagicConfigs.withMotionMagicJerk(100.0);

        var softLimitConfigs = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        softLimitConfigs.withForwardSoftLimitThreshold(MAX_POSITION);
        softLimitConfigs.withForwardSoftLimitEnable(false);
        softLimitConfigs.withReverseSoftLimitThreshold(MIN_POSITION);
        softLimitConfigs.withReverseSoftLimitEnable(false);

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

        supplyCurrentSignal = pivotMotor.getSupplyCurrent();
        appliedVoltsSignal = pivotMotor.getSupplyVoltage();
        tempSignal = pivotMotor.getDeviceTemp();


        setDefaultCommand(Commands.run(() -> holdCurrentPosition(), this));
  
    BaseStatusSignal.setUpdateFrequencyForAll(
    50.0, supplyCurrentSignal, appliedVoltsSignal, tempSignal
);
    }


    public void spinVoltage(double voltage) {
        pivotMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        // targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        pivotMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public double getError() {
        return targetPosition - getPosition();
    }

    public double getSupplyCurrent() {
        supplyCurrentSignal.refresh();
        return supplyCurrentSignal.getValueAsDouble();
    }

    public boolean atPosition(double targetPosition, double tolerance) {
        return Math.abs(getPosition() - targetPosition) < tolerance;
    }

    public void holdCurrentPosition() {
        pivotMotor.setControl(positionRequest.withPosition(getPosition()));
    }

    public void stop() {
        pivotMotor.stopMotor();
    }

@Override
public LoggableInputs log() {
    inputs.targetPosition    = targetPosition;
    inputs.positionRotations = getPosition();
    inputs.appliedVolts      = appliedVoltsSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.tempCelsius       = tempSignal.getValueAsDouble();
    inputs.positionError     = getError();
    inputs.atTarget          = atPosition(targetPosition, AT_TARGET_TOLERANCE);
    inputs.motorConnected    = BaseStatusSignal.isAllGood(
                                   supplyCurrentSignal, appliedVoltsSignal, tempSignal);
    return inputs;
}
}