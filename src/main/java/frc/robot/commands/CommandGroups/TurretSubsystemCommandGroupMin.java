package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.TurretCommands.HoodCommands.SetHoodPositionCommand;
import frc.robot.commands.TurretCommands.RotationCommands.TurretPositionCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVelocityCommand;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;

public class TurretSubsystemCommandGroupMin extends ParallelCommandGroup {
    public TurretSubsystemCommandGroupMin(RotateSubsystem turret, HoodSubsystem hood, ShooterSubsystem shooter) {
        addCommands(
            new TurretPositionCommand(turret, -1),
            new SetHoodPositionCommand(hood, 0.2),
            new ShooterVelocityCommand(shooter, 30)
        //     Commands.run(() -> 
        //         System.out.println("Turret Rotation: " + turret.getPosition() + 
        //                          " | Hood Position: " + hood.getPosition() + 
        //                          " | Shooter RPS: " + shooter.getVelocity()))

        );

    }
}