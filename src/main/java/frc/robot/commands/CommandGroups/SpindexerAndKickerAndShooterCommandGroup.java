package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexterSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVelocityCommand;

public class SpindexerAndKickerAndShooterCommandGroup extends ParallelCommandGroup {
    public SpindexerAndKickerAndShooterCommandGroup(KickerSubsystem kicker, SpindexterSubsystem spinDexer, ShooterSubsystem shooter) {
        addCommands(
            new KickerSpinCommand(kicker, -200),
            new SpindexerCommand(spinDexer, -40),
            new ShooterVelocityCommand(shooter, -200));
          Commands.run(() -> 
                 System.out.println("Kicker Position: " + kicker.getPosition() + 
                                  " | Spin RPS: " + spinDexer.getVelocity())
        );
        
    }
}

