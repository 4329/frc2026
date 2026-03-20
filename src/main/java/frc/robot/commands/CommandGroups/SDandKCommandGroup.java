package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexterSubsystem;

public class SDandKCommandGroup extends ParallelCommandGroup{
    public SDandKCommandGroup(KickerSubsystem kicker, SpindexterSubsystem spinDexer) {
        addCommands(
            new KickerSpinCommand(kicker, -200),
            new SpindexerCommand(spinDexer, -80));
    }
}
