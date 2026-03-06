package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;
import frc.robot.subsystems.SpinDexterSubsystem;

public class SpindexerCommand extends Command {
    private final SpinDexterSubsystem SpinDexer;
    private final double rotationsPerSecond;

    
    public SpindexerCommand(SpinDexterSubsystem SpinDexer, double rotationsPerSecond) {
        this.SpinDexer = SpinDexer;
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(SpinDexer);
    }


    @Override
    public void execute() {
        SpinDexer.setVelocity(rotationsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        SpinDexer.stop();
    }
    
}