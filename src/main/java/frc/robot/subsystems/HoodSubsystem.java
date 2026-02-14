package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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

// Create configuration object
TalonFXConfiguration config = new TalonFXConfiguration();

// Configure motor output
config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

// Configure PID gains (tune these values for your mechanism)
config.Slot0.kP = 8.0; // Start with this, tune as needed
config.Slot0.kI = 0.0;
config.Slot0.kD = 0.05; // Small D term helps with stability
config.Slot0.kV = 0.0; // Feedforward velocity term
config.Slot0.kS = 0.0; // Feedforward static friction term

// Configure feedback sensor
config.Feedback.SensorToMechanismRatio = 1.0; // Adjust based on your gearing

// Configure motion magic for smooth movement (optional but recommended)
// config.MotionMagic.MotionMagicCruiseVelocity = 2.0; // rotations per second
// config.MotionMagic.MotionMagicAcceleration = 4.0; // rotations per second^2
// config.MotionMagic.MotionMagicJerk = 40.0; // rotations per second^3

// Apply configuration
hoodMotor.getConfigurator().apply(config);

// Set initial position
hoodMotor.setPosition(0.0);

// Default command holds current position
setDefaultCommand(Commands.run(() -> holdPosition(), this));
}

/*
  Sets the hood to a target position with safety checks
  @param rotations Target position in rotations
 */
public void setPosition(double rotations) {
// Clamp position to safe range
double clampedPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
hoodMotor.setControl(positionRequest.withPosition(clampedPosition));
}

/*
  Holds the hood at its current position
 */
public void holdPosition() {
setPosition(getPosition());
}

/*
  Gets the current hood position
  @return Current position in rotations
 */
public double getPosition() {
return hoodMotor.getPosition().getValueAsDouble();
}

/*
  Gets the current velocity
  @return Current velocity in rotations per second
 */
public double getVelocity() {
return hoodMotor.getVelocity().getValueAsDouble();
}

/*
 Checks if hood is at the target position
  @param targetPosition Target position in rotations
 @param tolerance Tolerance in rotations
 @return true if within tolerance
 */
public boolean atPosition(double targetPosition, double tolerance) {
return Math.abs(getPosition() - targetPosition) < tolerance;
}

/*
  Stops the hood motor
 */
public void stop() {
hoodMotor.stopMotor();
}

@Override
public void periodic() {
// Log telemetry
Logger.recordOutput("Hood/Position", getPosition());
Logger.recordOutput("Hood/Velocity", getVelocity());
Logger.recordOutput("Hood/MotorCurrent", hoodMotor.getSupplyCurrent().getValueAsDouble());

// System.out.println("HOOD ROTATION POSITION: " + getPosition());
// System.out.println("TARGET POSITION: " + positionRequest.Position);

}
}