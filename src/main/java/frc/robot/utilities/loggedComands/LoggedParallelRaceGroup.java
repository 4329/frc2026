package frc.robot.utilities.loggedComands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.model.CommandLogEntry;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedParallelRaceGroup extends LoggedCommandComposer implements LoggableInputs {
    // LinkedHashSet guarantees we iterate over commands in the order they were
    // added
    private final Set<Command> m_commands = new LinkedHashSet<>();
    private boolean m_runWhenDisabled = true;
    private boolean m_finished = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    private final CommandLogEntry commandLogEntry;

    protected LoggedParallelRaceGroup(Command... commands) {
        commandLogEntry = new CommandLogEntry();
        addCommands(commands);
    }

    /**
     * Creates a new ParallelCommandRace. The given commands will be executed simultaneously, and will
     * "race to the finish" - the first command to finish ends the entire command, with all other
     * commands being interrupted.
     *
     * @param commands the commands to include in this composition.
     */
    @SuppressWarnings("this-escape")
    public LoggedParallelRaceGroup(String name, Command... commands) {
        this(commands);
        setName(name);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add to the group.
     */
    @SuppressWarnings("PMD.UseArraysAsList")
    public final void addCommands(Command... commands) {
        if (!m_finished) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running!");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
                throw new IllegalArgumentException(
                        "Multiple commands in a parallel composition cannot require the same subsystems");
            }
            m_commands.add(command);
            addRequirements(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public void initialize() {
        m_finished = false;
        for (Command command : m_commands) {
            commandLogEntry.put(command, "initialize()");
            command.initialize();
        }
    }

    @Override
    public void execute() {
        for (Command command : m_commands) {
            commandLogEntry.put(command, "execute()");
            command.execute();
            if (command.isFinished()) {
                m_finished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (Command command : m_commands) {
            commandLogEntry.put(command, "end(" + !command.isFinished() + ")");
            command.end(!command.isFinished());
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
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
        commandLogEntry.toLog(table);
    }
}
