package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.model.CommandLogEntry;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class CommandConsumingLogger implements Consumer<Command> {

    private final String message;
    private final CommandLogEntry commandLogEntry;

    public CommandConsumingLogger(String message, CommandLogEntry commandLogEntry) {
        this.message = message;
        this.commandLogEntry = commandLogEntry;
    }

    @Override
    public void accept(Command command) {
        commandLogEntry.put(command, message);
        Logger.processInputs("Command", commandLogEntry);
    }
}
