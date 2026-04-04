package frc.robot.commands.TurretCommands.HoodCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;

public class SetHoodPositionCommand extends Command {
    private final HoodSubsystem hood;
    private final DoubleSupplier targetPositionSupplier;
    private double targetPosition;
    private static final double TOLERANCE = 0.1;

    public SetHoodPositionCommand(HoodSubsystem hood, DoubleSupplier targetPositionSupplier) {
        this.hood = hood;
        this.targetPositionSupplier = targetPositionSupplier;
        addRequirements(hood);
    }

    public SetHoodPositionCommand(HoodSubsystem hood, double targetPosition) {
        this(hood, () -> targetPosition);
    }

    @Override
    public void initialize() {
        targetPosition = targetPositionSupplier.getAsDouble();
        hood.setPosition(targetPosition);
    }



    @Override
    public boolean isFinished() {
        return hood.atPosition(targetPosition, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        hood.holdPosition();
    }
}