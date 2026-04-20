package frc.robot.commands.CommandGroups;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.IntakeCommands.IntakePivotOscillateCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;
import yams.mechanisms.positional.Pivot;

public class SpindexerAndKickerCommandGroup extends ParallelCommandGroup {

    private static final InterpolatingDoubleTreeMap SPINDEXER_TABLE = new InterpolatingDoubleTreeMap();

    static{

        SPINDEXER_TABLE.put(2.2, 85.0);
        SPINDEXER_TABLE.put(2.5, 85.0);
        SPINDEXER_TABLE.put(2.8, 85.0);
        SPINDEXER_TABLE.put(3.1, 85.0);
        SPINDEXER_TABLE.put(3.4, 85.0);
        SPINDEXER_TABLE.put(3.7, 85.0);
        SPINDEXER_TABLE.put(4.0, 85.0);
        SPINDEXER_TABLE.put(4.3, 85.0);
        
    }

    public SpindexerAndKickerCommandGroup(SpindexterSubsystem spindexer, KickerSubsystem kicker, IntakePivotSubsystem pivot, IntakeSpinSubsystem spin) {
        addCommands(
            new SpindexerCommand(spindexer, 85),
            new KickerSpinCommand(kicker, 200),
            new IntakeSpinCommand(spin, 40),
            new IntakePivotOscillateCommand(pivot)
        );
    }
}
