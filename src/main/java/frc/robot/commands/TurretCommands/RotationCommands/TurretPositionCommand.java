package frc.robot.commands.TurretCommands.RotationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;

public class TurretPositionCommand extends Command {
    private final RotateSubsystem turretRotate;
    private final double targetPosition;
    private static final double TOLERANCE = 0.01;

    public TurretPositionCommand(RotateSubsystem turretRotate, double targetPosition) {
        this.turretRotate = turretRotate;
        this.targetPosition = targetPosition;
        addRequirements(turretRotate);
    }

       @Override
    public void initialize() {
        turretRotate.setPositionWithVelocity(targetPosition);
    }

    @Override
    public void execute() {
        turretRotate.setPositionWithVelocity(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return turretRotate.atPosition(targetPosition, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
    }
}