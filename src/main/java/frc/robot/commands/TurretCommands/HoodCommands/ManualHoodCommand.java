package frc.robot.commands.TurretCommands.HoodCommands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;

public class ManualHoodCommand extends Command {
    private static final double DUTY_CYCLE = 0.30;

    private final HoodSubsystem hood;
    private final boolean negative;

    private final DutyCycleOut request = new DutyCycleOut(0).withEnableFOC(true);

    public ManualHoodCommand(HoodSubsystem hood, boolean negative) {
        this.hood = hood;
        this.negative = negative;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.hoodMotor.setControl(request.withOutput(negative ? -DUTY_CYCLE : DUTY_CYCLE));
    }

    @Override
    public void end(boolean interrupted) {
        hood.holdPosition();
    }
}