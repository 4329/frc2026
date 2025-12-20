package frc.robot.utilities.loggedComands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.model.CommandLogEntry;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedParallelCommandGroup extends LoggedCommandComposer implements LoggableInputs {
    // maps commands in this composition to whether they are still running
    // LinkedHashMap guarantees we iterate over commands in the order they were
    // added (Note that
    // changing the value associated with a command does NOT change the order)
    private final Map<Command, Boolean> m_commands = new LinkedHashMap<>();
    private boolean m_runWhenDisabled = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    private final CommandLogEntry commandLogEntry;

    protected LoggedParallelCommandGroup(Command... commands) {
        commandLogEntry = new CommandLogEntry();
        addCommands(commands);
    }

    /**
     * Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The
     * command composition will finish when the last command finishes. If the composition is
     * interrupted, only the commands that are still running will be interrupted.
     *
     * @param commands the commands to include in this composition.
     */
    @SuppressWarnings("this-escape")
    public LoggedParallelCommandGroup(String name, Command... commands) {
        this(commands);
        setName(name);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add to the group.
     */
    public void addCommands(Command... commands) {
        if (m_commands.containsValue(true)) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
                throw new IllegalArgumentException(
                        "Multiple commands in a parallel composition cannot require the same subsystems");
            }
            m_commands.put(command, false);
            addRequirements(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public void initialize() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            commandRunning.getKey().initialize();
            commandRunning.setValue(true);
            commandLogEntry.put(commandRunning.getKey(), "initialize()");
        }
    }

    @Override
    public void execute() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (!commandRunning.getValue()) {
                continue;
            }
            commandRunning.getKey().execute();
            commandLogEntry.put(commandRunning.getKey(), "execute()");

            if (commandRunning.getKey().isFinished()) {
                commandRunning.getKey().end(false);
                commandRunning.setValue(false);

                commandLogEntry.put(commandRunning.getKey(), "end(false)");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
                if (commandRunning.getValue()) {
                    commandRunning.getKey().end(true);
                    commandLogEntry.put(commandRunning.getKey(), "end(true)");
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !m_commands.containsValue(true);
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_interruptBehavior;
    }

    @Override
    public void toLog(LogTable table) {
        commandLogEntry.toLog(table);
    }

    @Override
    public void fromLog(LogTable table) {
        commandLogEntry.fromLog(table);
    }
}
