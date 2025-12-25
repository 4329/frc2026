package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggingSubsystem extends SubsystemBase {

    private LoggedSubsystem[] subsystems;
    private int timer;

    private final int oftenness = 2;

    public LoggingSubsystem(LoggedSubsystem... subsystems) {
        this.subsystems = subsystems;
    }

    @Override
    public void periodic() {
        timer++;
        timer %= oftenness;

        for (int i = timer; i < subsystems.length; i += oftenness) {
            String name = subsystems[i].getNameLog();
            Logger.processInputs(name, subsystems[i].log());
        }
    }

    public static interface LoggedSubsystem {

        public LoggableInputs log();

        public default String getNameLog() {
            return getClass().getSimpleName();
        }
    }
}
