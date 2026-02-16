package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
public final TalonFX hoodMotor;
private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);

// Hood position limits
private static final double MIN_POSITION = 0.05;
private static final double MAX_POSITION = 0.5;

public HoodSubsystem() {
hoodMotor = new TalonFX(42);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(25);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.05);
        Slot0Configs.withKV(0.12);

        // ADD MOTION MAGIC CONFIGURATION
        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(3);   // rotations per second - tune this
        motionMagicConfigs.withMotionMagicAcceleration(8);   // rotations per second^2 - tune this
        motionMagicConfigs.withMotionMagicJerk(80);   // rotations per second^3 - tune this
        
        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);
        
        hoodMotor.getConfigurator().apply(motorOutputConfigs);
        hoodMotor.getConfigurator().apply(Slot0Configs);
        hoodMotor.getConfigurator().apply(motionMagicConfigs); // APPLY THIS
        hoodMotor.getConfigurator().apply(feedbackConfigs);

// Set initial position
        hoodMotor.setPosition(0.0);

        setDefaultCommand(Commands.run(() -> holdPosition(), this));
}

public void setPosition(double rotations) {
    double clampedPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
    hoodMotor.setControl(positionRequest.withPosition(clampedPosition));
}

public void holdPosition() {
    setPosition(getPosition());
}

public double getPosition() {
return hoodMotor.getPosition().getValueAsDouble();
}

public double getVelocity() {
return hoodMotor.getVelocity().getValueAsDouble();
}

public boolean atPosition(double targetPosition, double tolerance) {
return Math.abs(getPosition() - targetPosition) < tolerance;
}

public void stop() {
hoodMotor.stopMotor();
}

@Override
public void periodic() {
// Log telemetry
Logger.recordOutput("Hood/Position", getPosition());
Logger.recordOutput("Hood/Velocity", getVelocity());
Logger.recordOutput("Hood/MotorCurrent", hoodMotor.getSupplyCurrent().getValueAsDouble());

System.out.println("HOOD ROTATION: " + hoodMotor.getPosition());
// System.out.println("TARGET POSITION: " + positionRequest.Position);

}
}