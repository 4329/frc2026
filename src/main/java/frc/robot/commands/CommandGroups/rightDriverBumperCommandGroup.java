package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.IntakeCommands.IntakePivotCommand;
import frc.robot.commands.IntakeCommands.IntakePivotOscillateCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;

public class rightDriverBumperCommandGroup extends ParallelCommandGroup{
    public rightDriverBumperCommandGroup(SpindexterSubsystem spindexer, KickerSubsystem kicker, IntakePivotSubsystem pivot, IntakeSpinSubsystem spin) {
        addCommands(
            new SpindexerCommand(spindexer, 85),
            new KickerSpinCommand(kicker, 200),
            new IntakePivotCommand(pivot, 6.3),
            new IntakeSpinCommand(spin, 40)
        );
    }
}
