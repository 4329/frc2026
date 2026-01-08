package frc.robot.model;
import org.littletonrobotics.junction.AutoLog;

/**
 * The ExampleSubsystemLog class is a data model used for logging purposes in the robot code.
 * It is annotated with @AutoLog, which indicates that the fields of this class
 * will be automatically logged or processed by the logger.
 * 
 * This class contains a single field:
 * - exampleValue: A double value initialized to 6.7, which can represent a sample
 *   or placeholder value for demonstration purposes.
 * 
 * This class can be extended or modified to include additional fields for logging
 * other subsystem-specific data. A new model should be created for each subsystem.
 */
@AutoLog
public class TightlyCoupledLog {
    public double speed = 0;
}
