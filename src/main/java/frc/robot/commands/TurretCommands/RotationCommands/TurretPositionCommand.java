package frc.robot.commands.TurretCommands.RotationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;

public class TurretPositionCommand extends Command {
    private final RotateSubsystem turret;
    private final double targetPosition;
    private static final double TOLERANCE = 0.01;

    public TurretPositionCommand(RotateSubsystem turret, double targetPosition) {
        this.turret = turret;
        this.targetPosition = targetPosition;
        addRequirements(turret);
    }

       @Override
    public void initialize() {
        turret.setPositionWithVelocity(targetPosition);
    }

    @Override
    public void execute() {
        turret.setPositionWithVelocity(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return turret.atPosition(targetPosition, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
    }
}