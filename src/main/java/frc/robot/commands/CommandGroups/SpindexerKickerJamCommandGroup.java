package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexterSubsystem;

public class SpindexerKickerJamCommandGroup extends SequentialCommandGroup {

    private static final double SPINDEXER_SPIN = -40.0;
    private static final double KICKER_SPIN = -200.0;

    public SpindexerKickerJamCommandGroup(SpindexterSubsystem spindexer, KickerSubsystem kicker) {
        addCommands(

            Commands.parallel(
                // Commands.runOnce(() -> new IntakePivotCommand(pivot, DEPLOY_POS)),
                new SpindexerCommand(spindexer, SPINDEXER_SPIN)),
                new KickerSpinCommand(kicker, KICKER_SPIN));
    }
}
