package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakePivotCommand extends Command {
    private final IntakePivotSubsystem pivot;
    private final double targetPosition;
    private static final double TOLERANCE = 0.1;


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
        return Math.abs(pivot.getPosition() - targetPosition) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
