package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;

public class IntakePivotCommand extends Command {
    private final IntakePivotSubsystem pivot;
    private final double targetPosition;
    private static final double TOLERANCE = 0.3;


    public IntakePivotCommand(IntakePivotSubsystem pivot, double targetPosition) {
        this.pivot = pivot;
        this.targetPosition = targetPosition;
        addRequirements(pivot);
    }


    @Override
    public void initialize() {
        pivot.setPosition(targetPosition);
    }

    @Override
    public void execute() {
        pivot.setPosition(targetPosition);
        pivot.holdCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        return pivot.atPosition(targetPosition, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.holdCurrentPosition();
    }
}