package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretMotorSubsystem;

public class TurretPositionWithSpeedCommand extends Command {
    private final TurretMotorSubsystem turretMotor;
    private final double targetPosition;
    private static final double TOLERANCE = 0.1;


    public TurretPositionWithSpeedCommand(TurretMotorSubsystem turretMotor, double targetPosition) {
        this.turretMotor = turretMotor;
        this.targetPosition = targetPosition;
        addRequirements(turretMotor);
    }


    @Override
    public void execute() {
        turretMotor.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(turretMotor.getPosition() - targetPosition) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
