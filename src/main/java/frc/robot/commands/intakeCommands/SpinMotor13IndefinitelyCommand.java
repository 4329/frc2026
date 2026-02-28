package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinMotor13Subsystem;

public class SpinMotor13IndefinitelyCommand extends Command {
    private final SpinMotor13Subsystem spin13;
    private final double voltage;
    private final boolean reverse; // new flag

    public SpinMotor13IndefinitelyCommand(SpinMotor13Subsystem spinner, double voltage, boolean reverse) {
        this.spin13 = spinner;
        this.voltage = voltage;
        this.reverse = reverse;
        addRequirements(spinner);
    }

    @Override
    public void execute() {
        spin13.spinVoltage(reverse ? -voltage : voltage);
    }

    @Override
    public void end(boolean interrupted) {
        spin13.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs indefinitely until canceled
    }
}