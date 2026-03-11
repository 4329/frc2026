package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVelocityCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpinDexterSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;

public class SDandKCommandGroup extends ParallelCommandGroup{
    public SDandKCommandGroup(KickerSubsystem kicker, SpinDexterSubsystem spinDexer) {
        addCommands(
            new KickerSpinCommand(kicker, -200),
            new SpindexerCommand(spinDexer, -80));
    }
}
