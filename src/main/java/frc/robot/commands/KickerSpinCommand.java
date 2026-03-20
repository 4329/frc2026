package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

public class KickerSpinCommand extends Command {
    private final KickerSubsystem kicker;
    private final double rotationsPerSecond;

    
    public KickerSpinCommand(KickerSubsystem kicker, double rotationsPerSecond) {
        this.kicker = kicker;
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(kicker);
    }


    @Override
    public void execute() {
        kicker.setVelocity(rotationsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
    }
    
}