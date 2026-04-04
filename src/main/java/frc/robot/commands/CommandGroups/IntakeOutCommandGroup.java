package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakePivotCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;

public class IntakeOutCommandGroup extends SequentialCommandGroup {

    private static final double DEPLOY_POS = 6.3;
    private static final double SPIN_SPEED = 60.0;

    public IntakeOutCommandGroup(IntakePivotSubsystem pivot, IntakeSpinSubsystem spin) {
        addCommands(
            new IntakePivotCommand(pivot, DEPLOY_POS),

            Commands.parallel(
                // Commands.runOnce(() -> new IntakePivotCommand(pivot, DEPLOY_POS)),
                new IntakePivotCommand(pivot, DEPLOY_POS)),
                new IntakeSpinCommand(spin, SPIN_SPEED));
    }
}
