package frc.robot.utilities.loggedComands;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

public class LoggedWaitUntilCommand extends LoggedCommandComposer {
    private final BooleanSupplier m_condition;

    /**
     * Creates a new WaitUntilCommand that ends after a given condition becomes true.
     *
     * @param condition the condition to determine when to end
     */
    public LoggedWaitUntilCommand(BooleanSupplier condition) {
        m_condition = condition;
    }

    /**
     * Creates a new WaitUntilCommand that ends after a given match time.
     *
     * <p>NOTE: The match timer used for this command is UNOFFICIAL. Using this command does NOT
     * guarantee that the time at which the action is performed will be judged to be legal by the
     * referees. When in doubt, add a safety factor or time the action manually.
     *
     * @param time the match time after which to end, in seconds
     */
    public LoggedWaitUntilCommand(double time) {
        this(() -> Timer.getMatchTime() - time > 0);
    }

    @Override
    public boolean isFinished() {
        return m_condition.getAsBoolean();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
