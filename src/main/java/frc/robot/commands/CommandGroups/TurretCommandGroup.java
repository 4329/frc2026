package frc.robot.commands.CommandGroups;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SpindexterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurretCommandGroup extends SequentialCommandGroup {

    private static final InterpolatingDoubleTreeMap SHOOTER_TABLE   = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HOOD_TABLE      = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap SPINDEXER_TABLE = new InterpolatingDoubleTreeMap();

    static {
        // distance (m) -> shooter velocity (rotations/sec)
        SHOOTER_TABLE.put(1.6, 40.0);
        SHOOTER_TABLE.put(1.9, 40.0);
        SHOOTER_TABLE.put(2.2, 40.0);
        SHOOTER_TABLE.put(2.5, 45.0);
        SHOOTER_TABLE.put(2.8, 45.0);
        SHOOTER_TABLE.put(3.1, 50.0);
        SHOOTER_TABLE.put(3.4, 50.0);
        SHOOTER_TABLE.put(3.7, 55.0);


        // distance (m) -> hood position (rotations)
        HOOD_TABLE.put(1.6, 2.1);
        HOOD_TABLE.put(1.9, 2.8);
        HOOD_TABLE.put(2.2, 3.0);
        HOOD_TABLE.put(2.5, 3.3);
        HOOD_TABLE.put(2.8, 3.6);
        HOOD_TABLE.put(3.1, 3.8);
        HOOD_TABLE.put(3.4, 4.2);
        HOOD_TABLE.put(3.7, 4.5);

        // distance (m) -> spindexer speed (rotations/sec)
        SPINDEXER_TABLE.put(1.6, 85.0);
        SPINDEXER_TABLE.put(1.9, 85.0);
        SPINDEXER_TABLE.put(2.2, 85.0);
        SPINDEXER_TABLE.put(2.5, 85.0);
        SPINDEXER_TABLE.put(2.8, 85.0);
        SPINDEXER_TABLE.put(3.1, 85.0);
        SPINDEXER_TABLE.put(3.4, 85.0);
        SPINDEXER_TABLE.put(3.7, 85.0);
    }

    private static final double SHOOTER_TOLERANCE = 2.0;
    private static final double HOOD_TOLERANCE    = 0.1; 
    private static final double DEFAULT_DISTANCE  = 1.9;

    public TurretCommandGroup(
            HoodSubsystem hood,
            ShooterSubsystem shooter,
            SpindexterSubsystem spindexer,
            VisionSubsystem vision) {

        double[] lastKnownDistance = { DEFAULT_DISTANCE };

        Command spinUpAndAim = new Command() {
            double targetVelocity = 0;
            double targetHoodPos  = 0;

            {
                addRequirements(shooter, hood);
            }

            @Override
            public void initialize() {
                updateTargets();
            }

            @Override
            public void execute() {
                if (vision.hasTarget()) {
                    lastKnownDistance[0] = vision.getTargetDistance();
                }
                updateTargets();
                shooter.setVelocity(targetVelocity);
                hood.setPosition(targetHoodPos);
            }

            @Override
            public boolean isFinished() {
                boolean shooterReady = Math.abs(shooter.getVelocity() - targetVelocity) < SHOOTER_TOLERANCE;
                boolean hoodReady    = hood.atPosition(targetHoodPos, HOOD_TOLERANCE);
                return shooterReady && hoodReady;
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) {
                    shooter.stop();
                    hood.holdPosition();
                }
            }

            private void updateTargets() {
                targetVelocity = SHOOTER_TABLE.get(lastKnownDistance[0]);
                targetHoodPos  = HOOD_TABLE.get(lastKnownDistance[0]);
            }
        };

        Command feed = Commands.parallel(
            Commands.run(() -> {
                shooter.setVelocity(SHOOTER_TABLE.get(lastKnownDistance[0]));
                hood.setPosition(HOOD_TABLE.get(lastKnownDistance[0]));
            }, shooter, hood),

            Commands.run(() ->
                spindexer.setVelocity(SPINDEXER_TABLE.get(lastKnownDistance[0])),
            spindexer)
        );

        addCommands(spinUpAndAim, feed);
    }
}