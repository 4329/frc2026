package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;

public class SpindexerCommand extends Command {
    private final SpindexerSubsystem Spindexer;
    private final double rotationsPerSecond;

    
    public SpindexerCommand(SpindexerSubsystem Spindexer, double rotationsPerSecond) {
        this.Spindexer = Spindexer;
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(Spindexer);
    }


    @Override
    public void execute() {
        Spindexer.setVelocity(rotationsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        Spindexer.stop();
    }
    
}