package frc.robot.commands.CommandGroups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AimAtHubCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;

public class FullTurretCommandGroup extends ParallelCommandGroup {

    private static final InterpolatingDoubleTreeMap SHOOTER_TABLE = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HOOD_TABLE    = new InterpolatingDoubleTreeMap();

    static {
        // distance (m) -> shooter velocity (rotations/sec)
        SHOOTER_TABLE.put(2.2 - 0.6, 40.0);
        SHOOTER_TABLE.put(2.5 - 0.6, 40.0);
        SHOOTER_TABLE.put(2.8 - 0.6, 40.0);
        SHOOTER_TABLE.put(3.1 - 0.6, 45.0);
        SHOOTER_TABLE.put(3.4 - 0.6, 45.0);
        SHOOTER_TABLE.put(3.7 - 0.6, 50.0);
        SHOOTER_TABLE.put(4.0 - 0.6, 50.0);
        SHOOTER_TABLE.put(4.3 - 0.6, 55.0);

        // distance (m) -> hood position (rotations)
        HOOD_TABLE.put(2.2 - 0.6, 2.1);
        HOOD_TABLE.put(2.5 - 0.6, 2.8);
        HOOD_TABLE.put(2.8 - 0.6, 3.0);
        HOOD_TABLE.put(3.1 - 0.6, 3.3);
        HOOD_TABLE.put(3.4 - 0.6, 3.6);
        HOOD_TABLE.put(3.7 - 0.6, 3.8);
        HOOD_TABLE.put(4.0 - 0.6, 4.2);
        HOOD_TABLE.put(4.3 - 0.6, 4.5);
    }

    private static final double DEFAULT_DISTANCE = 2.2 - 0.6;

    public FullTurretCommandGroup(
            HoodSubsystem hood,
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        double[] lastKnownDistance = { DEFAULT_DISTANCE };

        addCommands(
            new AimAtHubCommand(drivetrain, xSupplier, ySupplier),

            Commands.run(() -> {
                if (vision.hasTarget()) {
                    lastKnownDistance[0] = vision.getTargetDistance();
                }
                shooter.setVelocity(SHOOTER_TABLE.get(lastKnownDistance[0]));
                hood.setPosition(HOOD_TABLE.get(lastKnownDistance[0]));
            }, shooter, hood)
        );
    }
}