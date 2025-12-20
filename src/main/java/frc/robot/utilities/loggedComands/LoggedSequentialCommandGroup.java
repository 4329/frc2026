// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.loggedComands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.model.CommandLogEntry;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A command composition that runs a list of commands in sequence.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class LoggedSequentialCommandGroup extends LoggedCommandComposer implements LoggableInputs {
    private final List<Command> m_commands = new ArrayList<>();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    private CommandLogEntry commandLogEntry;

    protected LoggedSequentialCommandGroup(Command... commands) {
        commandLogEntry = new CommandLogEntry();
        addCommands(commands);
    }

    /**
     * Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the
     * composition finishing when the last command finishes.
     *
     * @param commands the commands to include in this composition.
     */
    @SuppressWarnings("this-escape")
    public LoggedSequentialCommandGroup(String name, Command... commands) {
        this(commands);
        setName(name);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add, in order of execution.
     */
    @SuppressWarnings("PMD.UseArraysAsList")
    public final void addCommands(Command... commands) {
        if (m_currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (int i = 0; i < commands.length; i++) {
            commands[i].setName(i + " " + commands[i].getName());
            m_commands.add(commands[i]);
            // commands[i] = commands[i].withTimeout(1);
            addRequirements(commands[i].getRequirements());
            m_runWhenDisabled &= commands[i].runsWhenDisabled();
            if (commands[i].getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public void initialize() {
        m_currentCommandIndex = 0;

        if (!m_commands.isEmpty()) {
            m_commands.get(0).initialize();
            commandLogEntry.put(m_commands.get(0), "initialize()");
        }
    }

    @Override
    public void execute() {
        if (m_commands.isEmpty()) {
            return;
        }

        Command currentCommand = m_commands.get(m_currentCommandIndex);

        currentCommand.execute();
        commandLogEntry.put(currentCommand, "execute()");

        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            commandLogEntry.put(currentCommand, "end(false)");
            m_currentCommandIndex++;

            if (m_currentCommandIndex < m_commands.size()) {
                m_commands.get(m_currentCommandIndex).initialize();
                commandLogEntry.put(m_commands.get(m_currentCommandIndex), "initialize()");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted
                && !m_commands.isEmpty()
                && m_currentCommandIndex > -1
                && m_currentCommandIndex < m_commands.size()) {
            m_commands.get(m_currentCommandIndex).end(true);
            commandLogEntry.put(m_commands.get(m_currentCommandIndex), "end(true)");
        }
        m_currentCommandIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return m_currentCommandIndex == m_commands.size();
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
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("index", () -> m_currentCommandIndex, null);
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
