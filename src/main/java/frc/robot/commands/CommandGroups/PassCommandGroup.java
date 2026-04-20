package frc.robot.commands.CommandGroups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AimAtPassCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;

public class PassCommandGroup extends ParallelCommandGroup {

    private static final double PASS_SHOOTER_VELOCITY = 60.0; // rotations/sec
    private static final double PASS_HOOD_ANGLE       = 6.0;  // rotations

    public PassCommandGroup(
            HoodSubsystem hood,
            ShooterSubsystem shooter,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        addCommands(
            new AimAtPassCommand(drivetrain, xSupplier, ySupplier),

            Commands.run(() -> {
                shooter.setVelocity(PASS_SHOOTER_VELOCITY);
                hood.setPosition(PASS_HOOD_ANGLE);
            }, shooter, hood)
        );
    }
}