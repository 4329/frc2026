package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;

public class IntakeSpinCommand extends Command {
    private final IntakeSpinSubsystem spinIntake;
    private final double rotationsPerSecond;

    
    public IntakeSpinCommand(IntakeSpinSubsystem spinIntake, double rotationsPerSecond) {
        this.spinIntake = spinIntake;
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(spinIntake);
    }


    @Override
    public void execute() {
        spinIntake.setVelocity(rotationsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        spinIntake.stop();
    }
    
}