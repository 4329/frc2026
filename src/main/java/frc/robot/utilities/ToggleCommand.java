package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;

public class ToggleCommand extends LoggedCommandComposer {

    private boolean on;
    private Command child;

    public ToggleCommand(Command child) {
        // child being canceled when disable ends is not a good
        // is there a better solution for this WIP
        this.child = child.ignoringDisable(true);
        setName("Toggle(" + child.getName() + ")");
    }

    @Override
    public void initialize() {
        if (on) child.cancel();
        else child.schedule();

        on = !on;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
