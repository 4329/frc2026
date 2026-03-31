package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;

public class IntakePivotCommand extends Command {
    private final IntakePivotSubsystem pivot;
    // private final double targetPosition;
    private final double targetVoltage;
    private static final double TOLERANCE = 0.3;


    public IntakePivotCommand(IntakePivotSubsystem pivot, double targetVoltage) {
        this.pivot = pivot;
        this.targetVoltage = targetVoltage;
        addRequirements(pivot);
    }


    @Override
    public void initialize() {
        // pivot.setPosition(targetPosition);
        pivot.spinVoltage(targetVoltage);
    }

    @Override
    public void execute() {
        // System.out.println(pivot.getError());
        // pivot.setPosition(targetPosition);
        pivot.spinVoltage(targetVoltage);
    }

    @Override
    public boolean isFinished() {
        // return pivot.atPosition(targetPosition, TOLERANCE);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.holdCurrentPosition();
    }
}