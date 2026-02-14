package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodPositionCommand extends Command {
    private final HoodSubsystem hood;
    private final double targetPosition;
    private static final double TOLERANCE = 0.000000001; // 0.01 rotations tolerance

    /**
     * Command to move hood to a specific position
     * @param hood Hood subsystem
     * @param targetPosition Target position in rotations (0.0 to 0.5)
     */
    public SetHoodPositionCommand(HoodSubsystem hood, double targetPosition) {
        this.hood = hood;
        this.targetPosition = targetPosition;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPosition(targetPosition);
    }

    @Override
    public void execute() {
        hood.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return hood.atPosition(targetPosition, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        // Hold position when command ends
        hood.holdPosition();
    }
}