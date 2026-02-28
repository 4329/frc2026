package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpinMotor13Subsystem;
import frc.robot.subsystems.KickerSubsystem;

public class KickerCommand extends Command {
    private final KickerSubsystem kicker;
    private final double voltage;
    private final boolean reverse; // new flag

    public KickerCommand(KickerSubsystem spinner, double voltage, boolean reverse) {
        this.kicker = spinner;
        this.voltage = voltage;
        this.reverse = reverse;
        addRequirements(spinner);
    }

    @Override
    public void execute() {
        kicker.spinVoltage(reverse ? -voltage : voltage);
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs indefinitely until canceled
    }
}