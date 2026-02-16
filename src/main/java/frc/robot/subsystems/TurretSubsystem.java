package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0); // ADD THIS

    private static final double MIN_POSITION = -1.2;
    private static final double MAX_POSITION = 1.2;

    public TurretSubsystem() {
        turretMotor = new TalonFX(41);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(1);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.02);
        Slot0Configs.withKV(0.12);
        // Slot0Configs.withKS(0.05);

        // ADD MOTION MAGIC CONFIGURATION
        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(3); // rotations per second - tune this
        motionMagicConfigs.withMotionMagicAcceleration(10);   // rotations per second^2 - tune this
        motionMagicConfigs.withMotionMagicJerk(50);          // rotations per second^3 - tune this
        
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
        turretMotor.getConfigurator().apply(motionMagicConfigs); // APPLY THIS
        turretMotor.getConfigurator().apply(feedbackConfigs);
        turretMotor.getConfigurator().apply(softLimitsConfigs);

        turretMotor.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    public void spinVoltage(double voltage) {
        turretMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        // turretMotor.setControl(positionRequest.withPosition(rotations));
        double clampedPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        turretMotor.setControl(positionRequest.withPosition(clampedPosition));
    }

    public void setVelocity(double rotationsPerSecond) {
        turretMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    public void setPositionWithVelocity(double rotations) {
        double clampedPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        turretMotor.setControl(motionMagicRequest.withPosition(clampedPosition));    
    }

    public double getPosition() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition(double targetPosition, double tolerance) {
    return Math.abs(getPosition() - targetPosition) < tolerance;
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    // @Override
    // public void periodic() {
    //     System.out.println("TURRET ROTATION POSITION: " + getPosition());
    //     super.periodic();
    // }
}
