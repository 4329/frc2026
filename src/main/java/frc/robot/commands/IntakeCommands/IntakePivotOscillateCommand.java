package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;

public class IntakePivotOscillateCommand extends Command {

    private static final double POSITION_A = 1.5;
    private static final double POSITION_B = 3.0;
    private static final double AT_TARGET_TOLERANCE = 0.1;

    private final IntakePivotSubsystem pivot;
    private double currentTarget;

    public IntakePivotOscillateCommand(IntakePivotSubsystem pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        currentTarget = POSITION_A;
        pivot.setPosition(currentTarget);
    }

    @Override
    public void execute() {
        pivot.holdCurrentPosition();

        if (pivot.atPosition(currentTarget, AT_TARGET_TOLERANCE)) {
            currentTarget = (currentTarget == POSITION_A) ? POSITION_B : POSITION_A;
            pivot.setPosition(currentTarget);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}