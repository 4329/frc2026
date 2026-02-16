package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretStuffCommandGroupMax extends ParallelCommandGroup {
    public TurretStuffCommandGroupMax(TurretSubsystem turret, HoodSubsystem hood, ShooterSubsystem shooter) {
        addCommands(
            new TurretPositionWithSpeedCommand(turret, 1),
            new SetHoodPositionCommand(hood, 0.45),
            new ShooterVelocityCommand(shooter, 100)
        );
        System.out.println("Turret Rotation: " + turret.getPosition());
        System.out.println("Hood Position: " + hood.getPosition());
        System.out.println("Shooter RPS: " + shooter.getVelocity());
    }

    
}