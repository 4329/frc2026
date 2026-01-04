// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.model.ExampleSubsystemLogAutoLogged;


// Implements LoggedSubsystem to enable logging
public class ExampleSubsystem extends SubsystemBase implements LoggedSubsystem { 

  // Create an instance of the log model
  private ExampleSubsystemLogAutoLogged exampleSubsystemAutoLogged = new ExampleSubsystemLogAutoLogged();

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
 * Updates and returns the loggable inputs for this subsystem.
 * <p>
 * This method populates the {@link ExampleSubsystemLogAutoLogged} instance with current
 * subsystem data (e.g., setting {@code exampleValue} to 6.7) and returns it for logging purposes.
 * It is required by the {@link LoggedSubsystem} interface to ensure telemetry data is recorded every loop.
 *
 * @return The updated {@link LoggableInputs} object containing the current state of the subsystem.
 */
  @Override
  public LoggableInputs log() {
    exampleSubsystemAutoLogged.exampleValue = 6.7;
    return exampleSubsystemAutoLogged;
  }
}
