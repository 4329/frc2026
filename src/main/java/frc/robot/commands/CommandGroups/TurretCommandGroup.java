package frc.robot.commands.CommandGroups;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurretCommandGroup extends Command {
    private static final InterpolatingDoubleTreeMap SHOOTER_TABLE = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HOOD_TABLE = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap SPINDEXER_TABLE = new InterpolatingDoubleTreeMap();

    static {
        // distance (m) -> shooter velocity (rotations/sec)
        SHOOTER_TABLE.put(1.29,  -60.0);


        // distance (m) -> hood position (rotations)
        HOOD_TABLE.put(1.29, 0.10);



        // distance (m) -> spindexer speed (rotations/sec)
        SPINDEXER_TABLE.put(1.29, 90.0);

    }

    private static final double SHOOTER_TOLERANCE = 2.0;   // rotations/sec
    private static final double HOOD_TOLERANCE    = 0.01;  // rotations

    private static final double DEFAULT_DISTANCE  = 1.0;

    private final HoodSubsystem    hood;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem  vision;

    private double lastKnownDistance = DEFAULT_DISTANCE;
    private double targetVelocity;
    private double targetHoodPos;

    public TurretCommandGroup(HoodSubsystem hood, ShooterSubsystem shooter, VisionSubsystem vision) {
        this.hood    = hood;
        this.shooter = shooter;
        this.vision  = vision;
        addRequirements(hood, shooter);
    }

    @Override
    public void initialize() {
        updateTargetsFromDistance();
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            lastKnownDistance = vision.getTargetDistance();
        }

        updateTargetsFromDistance();
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
        hood.holdPosition();
    }


    private void updateTargetsFromDistance() {
        targetVelocity = SHOOTER_TABLE.get(lastKnownDistance);
        targetHoodPos  = HOOD_TABLE.get(lastKnownDistance);
    }
}