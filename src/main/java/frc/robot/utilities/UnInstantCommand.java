package frc.robot.utilities;

import frc.robot.utilities.loggedComands.LoggedCommandComposer;

public class UnInstantCommand extends LoggedCommandComposer {

    Runnable toRun;

    public UnInstantCommand(String name, Runnable toRun) {
        setName(name);
        this.toRun = toRun;
    }

    @Override
    public void execute() {
        toRun.run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
