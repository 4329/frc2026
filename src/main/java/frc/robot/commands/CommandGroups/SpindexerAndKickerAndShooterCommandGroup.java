package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpinDexterSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVelocityCommand;

public class SpindexerAndKickerAndShooterCommandGroup extends ParallelCommandGroup {
    public SpindexerAndKickerAndShooterCommandGroup(KickerSubsystem kicker, SpinDexterSubsystem spinDexer, ShooterSubsystem shooter) {
        addCommands(
            new KickerSpinCommand(kicker, 5.9),
            new SpindexerCommand(spinDexer, 5.9),
            new ShooterVelocityCommand(shooter, 100));
          Commands.run(() -> 
                 System.out.println("Kicker Position: " + kicker.getPosition() + 
                                  " | Spin RPS: " + spinDexer.getVelocity())
        );
        
    }

    
}