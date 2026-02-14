package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPositionWithSpeedCommand extends Command {
    private final TurretSubsystem turret;
    private final double targetPosition;
    private static final double TOLERANCE = 0.01;

    public TurretPositionWithSpeedCommand(TurretSubsystem turret, double targetPosition) {
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