package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakePivotCommand;
import frc.robot.commands.IntakeCommands.IntakeZeroCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;

public class IntakePivotToZeroCommandGroup extends SequentialCommandGroup {
    public IntakePivotToZeroCommandGroup(IntakePivotSubsystem pivot) {
        addCommands(
            new IntakePivotCommand(pivot, 3.0),
            new IntakeZeroCommand(pivot)
        );
    }
}
