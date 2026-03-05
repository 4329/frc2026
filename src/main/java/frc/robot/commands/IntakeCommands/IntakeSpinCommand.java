package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;

public class IntakeSpinCommand extends Command {
    private final IntakeSpinSubsystem spin;
    private final double rotationsPerSecond;

    
    public IntakeSpinCommand(IntakeSpinSubsystem spin, double rotationsPerSecond) {
        this.spin = spin;
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(spin);
    }


    @Override
    public void execute() {
        spin.setVelocity(rotationsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        spin.stop();
    }
    
}
