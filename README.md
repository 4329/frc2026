# FRC Robot Code Documentation

This repository contains the robot code for the 2026 FRC season, built using WPILib, AdvantageKit, and the `yams` library. The core logic resides in `src/main/java/frc/robot`.

## Directory Structure

The code is organized into the following packages within `src/main/java/frc/robot`:

- **`commands`**: Contains robot commands (e.g., `Shoot`, `Autos`).
- **`subsystems`**: Contains robot subsystems (e.g., `ExampleShooter`) and the `LoggingSubsystem`.
- **`model`**: Contains data models annotated for AdvantageKit logging (e.g., `ShooterLog`).
- **`utilities`**: Helper classes, including the configuration system (`Config`, `HoorayConfig`).
- **Root**: `Robot.java`, `RobotContainer.java`, `Constants.java`, and `Main.java`.

## Subsystem & Command Framework

The project uses the standard **WPILib Command-based framework**.

- **Subsystems**: Encapsulate hardware and logic (e.g., `ExampleShooter`). They extend `SubsystemBase`.
  - Many subsystems implement the **`LoggedSubsystem`** interface, designed to work with the `LoggingSubsystem` for centralized data collection.
  - Integration with the **`yams`** library allows for advanced motor control wrapping (`SmartMotorController`) and mechanism simulation (`FlyWheel`).
- **Commands**: Define robot actions. Dependencies are injected via constructors to ensure modularity.
- **LoggingSubsystem**: A specialized subsystem designed to iterate through registered `LoggedSubsystem` instances and process their logs. It employs **time-slicing** (via an `oftenness` parameter) to distribute the CPU overhead of logging across multiple robot cycles.

## Logging (AdvantageKit)

The robot uses **AdvantageKit** to separate data flow provided by the IO layer from the robot logic, enabling high-fidelity replay simulation.

1.  **Modeling**: Data to be logged is defined in **`model`** classes (e.g., `ShooterLog`).
    - The **`@AutoLog`** annotation automatically generates a `LoggableInputs` implementation (e.g., `ShooterLogAutoLogged`).
2.  **Telemetry**: Subsystems update these model objects during execution. The logging system then records this state.
3.  **Modes**: `Robot.java` handles initialization for different environments:
    - **REAL**: Logs to USB (if available) or internal storage using `WPILOGWriter` and publishes to NetworkTables (`NT4Publisher`).
    - **SIM**: Runs robot logic in simulation.
    - **REPLAY**: Reads from a log file to rerun code against recorded data.

## Configuration & Constants

Configuration is split between static constants and dynamic runtime configuration:

- **`Constants.java`**: Stores static values (e.g., specific voltage compensation) and acts as the bridge to retrieval of dynamic values.
- **`utilities/HoorayConfig.java`**: A singleton that manages loading configuration.
  - It attempts to parse a JSON file (e.g., from `/home/lvuser/proto`) into the **`Config`** object.
  - This allows the same code deployment to adapt ports, offsets, and settings for different physical robots (e.g., "Proto" vs "Comp" bot).

## Core Classes

- **`Robot.java`**: The main robot class. It initializes the AdvantageKit `Logger` and Phoenix 6 `SignalLogger`, determines the runtime mode (Real/Sim/Replay), and manages the main loop.
- **`RobotContainer.java`**: Serves as the dependency injection container.
  - Initializes subsystems and controllers.
  - Configures button bindings (e.g., binding the `Shoot` command to a controller button).
  - Sets up the autonomous command chooser.
