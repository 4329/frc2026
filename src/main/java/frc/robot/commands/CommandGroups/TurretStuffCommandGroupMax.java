package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.TurretCommands.HoodCommands.SetHoodPositionCommand;
import frc.robot.commands.TurretCommands.RotationCommands.TurretRotationPositionWithSpeedCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVelocityCommand;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;

public class TurretStuffCommandGroupMax extends ParallelCommandGroup {
    public TurretStuffCommandGroupMax(RotateSubsystem turret, HoodSubsystem hood, ShooterSubsystem shooter) {
        addCommands(
            new TurretRotationPositionWithSpeedCommand(turret, 1),
            new SetHoodPositionCommand(hood, 0.45),
            new ShooterVelocityCommand(shooter, 100)
        );
        System.out.println("Turret Rotation: " + turret.getPosition());
        System.out.println("Hood Position: " + hood.getPosition());
        System.out.println("Shooter RPS: " + shooter.getVelocity());
    }

    
}