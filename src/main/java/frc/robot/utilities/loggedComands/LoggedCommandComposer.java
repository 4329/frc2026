package frc.robot.utilities.loggedComands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.UnInstantCommand;
import java.util.function.BooleanSupplier;

public class LoggedCommandComposer extends Command {
    public LoggedWrapperCommand withNameLog(String name) {
        LoggedWrapperCommand wrapper = new LoggedWrapperCommand(this) {};

        wrapper.setName(name);
        return wrapper;
    }

    public LoggedSequentialCommandGroup andThenLog(Command next) {
        return new LoggedSequentialCommandGroup(this.getName() + "," + next.getName(), this, next);
    }

    public LoggedParallelRaceGroup raceWithLog(String name, Command... parallel) {
        LoggedParallelRaceGroup group = new LoggedParallelRaceGroup(name, this);
        group.addCommands(parallel);
        return group;
    }

    public LoggedParallelRaceGroup untilLog(BooleanSupplier condition) {
        return raceWithLog("Until(" + getName() + ")", new LoggedWaitUntilCommand(condition));
    }

    public LoggedParallelRaceGroup withTimeoutLog(double seconds) {
        return raceWithLog("Timeout(" + getName() + ")", new LoggedWaitCommand(seconds));
    }

    public LoggedRepeatCommand repeatedlyLog() {
        return new LoggedRepeatCommand(this);
    }

    public LoggedWrapperCommand ignoringDisableLog(boolean doesRunWhenDisabled) {
        return new LoggedWrapperCommand(this) {
            @Override
            public boolean runsWhenDisabled() {
                return doesRunWhenDisabled;
            }
        };
    }

    public LoggedWrapperCommand whileLog(BooleanSupplier condition) {
        return new LoggedWrapperCommand(this) {
            @Override
            public boolean isFinished() {
                return !condition.getAsBoolean();
            }
        };
    }

    public LoggedParallelRaceGroup onlyWhileLog(BooleanSupplier condition) {
        return untilLog(() -> !condition.getAsBoolean());
    }

    public LoggedConditionalCommand unlessLog(BooleanSupplier condition) {
        return new LoggedConditionalCommand(new UnInstantCommand("", () -> {}), this, condition);
    }

    public LoggedConditionalCommand onlyIfLog(BooleanSupplier condition) {
        return unlessLog(() -> !condition.getAsBoolean());
    }

    public LoggedParallelCommandGroup alongWithLog(Command... parallel) {
        LoggedParallelCommandGroup group = new LoggedParallelCommandGroup(this);
        group.addCommands(parallel);
        return group;
    }
}
