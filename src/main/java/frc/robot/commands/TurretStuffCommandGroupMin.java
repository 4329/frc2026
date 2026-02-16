package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretStuffCommandGroupMin extends ParallelCommandGroup {
    public TurretStuffCommandGroupMin(TurretSubsystem turret, HoodSubsystem hood, ShooterSubsystem shooter) {
        addCommands(
            new TurretPositionWithSpeedCommand(turret, -1),
            new SetHoodPositionCommand(hood, 0.2),
            new ShooterVelocityCommand(shooter, 30),
            Commands.run(() -> 
                System.out.println("Turret Rotation: " + turret.getPosition() + 
                                 " | Hood Position: " + hood.getPosition() + 
                                 " | Shooter RPS: " + shooter.getVelocity()))

        );

    }
}