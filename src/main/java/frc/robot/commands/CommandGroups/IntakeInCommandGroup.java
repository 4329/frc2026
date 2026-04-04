package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakePivotCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.commands.IntakeCommands.IntakeZeroCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;

public class IntakeInCommandGroup extends SequentialCommandGroup {

    private static final double MID_POS = 3.0;

    public IntakeInCommandGroup(IntakePivotSubsystem pivot) {
        addCommands(
            new IntakePivotCommand(pivot, MID_POS)
        );
    }
    
}
