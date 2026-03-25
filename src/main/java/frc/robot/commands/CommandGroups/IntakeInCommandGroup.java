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

    private static final double STOW_POS = 0.0;

    public IntakeInCommandGroup(IntakePivotSubsystem pivot, IntakeSpinSubsystem spin) {
        addCommands(

        Commands.parallel(
            Commands.runOnce(() -> spin.stop(), spin),
            new IntakePivotCommand(pivot, STOW_POS)
        ),

        new IntakeZeroCommand(pivot)
        );
    }
    
}
