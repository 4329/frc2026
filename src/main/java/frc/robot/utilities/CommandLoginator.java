package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.model.CommandLogEntry;

public class CommandLoginator {
    private CommandLogEntry commandLogEntry;

    public CommandLoginator() {
        commandLogEntry = new CommandLogEntry();
        init();
    }

    private void init() {
        CommandScheduler.getInstance()
                .onCommandInitialize(new CommandConsumingLogger("initialize()", commandLogEntry));
        CommandScheduler.getInstance()
                .onCommandExecute(new CommandConsumingLogger("execute()", commandLogEntry));
        CommandScheduler.getInstance()
                .onCommandFinish(new CommandConsumingLogger("end(false)", commandLogEntry));
        CommandScheduler.getInstance()
                .onCommandInterrupt(new CommandConsumingLogger("end(true)", commandLogEntry));
    }
}
