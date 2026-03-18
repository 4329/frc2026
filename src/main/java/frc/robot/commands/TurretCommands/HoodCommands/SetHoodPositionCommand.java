package frc.robot.commands.TurretCommands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;

public class SetHoodPositionCommand extends Command {
    private final HoodSubsystem hood;
    private final double targetPosition;
    private static final double TOLERANCE = 0.001;

    public SetHoodPositionCommand(HoodSubsystem hood, double targetPosition) {
        this.hood = hood;
        this.targetPosition = targetPosition;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPosition(targetPosition);
    }

    // @Override
    // public void execute() {
    //     hood.setPosition(targetPosition);
    // }

    @Override
    public boolean isFinished() {
        return hood.atPosition(targetPosition, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        hood.holdPosition();
    }
}