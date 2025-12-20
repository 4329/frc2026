package frc.robot.utilities.loggedComands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedRepeatCommand extends LoggedCommandComposer implements LoggableInputs {
    private final Command m_command;
    private boolean m_ended;

    private String message;

    /**
     * Creates a new RepeatCommand. Will run another command repeatedly, restarting it whenever it
     * ends, until this command is interrupted.
     *
     * @param command the command to run repeatedly
     */
    @SuppressWarnings("this-escape")
    public LoggedRepeatCommand(Command command) {
        m_command = requireNonNullParam(command, "command", "RepeatCommand");
        CommandScheduler.getInstance().registerComposedCommands(command);
        addRequirements(command.getRequirements());
        setName("LoggedRepeat(" + command.getName() + ")");
    }

    @Override
    public void initialize() {
        m_ended = false;
        m_command.initialize();
        message = "initialize()";
    }

    @Override
    public void execute() {
        if (m_ended) {
            m_ended = false;
            m_command.initialize();
            message = "initialize()";
        }
        m_command.execute();
        if (m_command.isFinished()) {
            // restart command
            m_command.end(false);
            message = "end(false)";
            m_ended = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure we didn't already call end() (which would happen if the command
        // finished in the
        // last call to our execute())
        if (!m_ended) {
            m_command.end(interrupted);
            message = "end(" + interrupted + ")";
            m_ended = true;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_command.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_command.getInterruptionBehavior();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("command", m_command::getName, null);
    }

    @Override
    public void toLog(LogTable table) {
        if (m_command instanceof LoggableInputs) ((LoggableInputs) m_command).toLog(table);
        else table.put(m_command.getName(), message);
    }

    @Override
    public void fromLog(LogTable table) {
        if (m_command instanceof LoggableInputs) ((LoggableInputs) m_command).fromLog(table);
        else message = table.get(getName(), message);
    }
}
