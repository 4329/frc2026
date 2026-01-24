package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;

public class PositionSpinNEO550Command extends Command {
    private final NEO550ThroughTalonFXSSubsytem spinner;
    private final double targetPosition;
    private static final double TOLERANCE = 0.1;


    public PositionSpinNEO550Command(NEO550ThroughTalonFXSSubsytem spinner, double targetPosition) {
        this.spinner = spinner;
        this.targetPosition = targetPosition;
        addRequirements(spinner);
    }


    @Override
    public void execute() {
        spinner.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(spinner.getPosition() - targetPosition) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
