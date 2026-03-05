package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;

public class IntakePivotCommand extends Command {
    private final IntakePivotSubsystem pivot;
    private final double targetPosition;

    public IntakePivotCommand(IntakePivotSubsystem pivot, double targetPosition) {
        this.pivot = pivot;
        this.targetPosition = targetPosition;
        addRequirements(pivot);
    }


    @Override
    public void execute() {
        pivot.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setPosition(0);
    }
}
