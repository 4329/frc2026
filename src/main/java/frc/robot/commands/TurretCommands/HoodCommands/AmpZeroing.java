package frc.robot.commands.TurretCommands.HoodCommands;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;

public class AmpZeroing extends Command {

    // Flip to -1.0 if the hood moves the wrong direction
    private static final double HOMING_VOLTAGE      = 1.0;

    // Amps at hard stop stall — lower if it never triggers, raise if it false-triggers
    private static final double AMP_SPIKE_THRESHOLD = 25.0;

    // Consecutive 20ms loops above threshold before latching (~100ms debounce)
    private static final int SPIKE_LOOP_COUNT = 5;

    private final HoodSubsystem hood;
    private final VoltageOut homingRequest = new VoltageOut(HOMING_VOLTAGE).withEnableFOC(true);

    private int     spikeLoops   = 0;
    private boolean spikeLatched = false;

    public AmpZeroing(HoodSubsystem hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        spikeLoops   = 0;
        spikeLatched = false;

        // 1. Disable soft limits — block until confirmed over CAN before moving
        var limitConfigs = new SoftwareLimitSwitchConfigs();
        limitConfigs.withForwardSoftLimitEnable(false);
        limitConfigs.withForwardSoftLimitThreshold(0.0);
        limitConfigs.withReverseSoftLimitEnable(false);
        limitConfigs.withReverseSoftLimitThreshold(0.0);
        hood.hoodMotor.getConfigurator().apply(limitConfigs, 0.050);

        // 2. Drive toward hard stop
        hood.hoodMotor.setControl(homingRequest);
    }

    @Override
    public void execute() {
        // Re-command every loop so nothing can interrupt the voltage request
        hood.hoodMotor.setControl(homingRequest);

        hood.hoodMotor.getSupplyCurrent().refresh();
        double amps = hood.hoodMotor.getSupplyCurrent().getValueAsDouble();

        if (amps >= AMP_SPIKE_THRESHOLD) {
            spikeLoops++;
            if (spikeLoops >= SPIKE_LOOP_COUNT) {
                spikeLatched = true;
            }
        } else {
            spikeLoops = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return spikeLatched;
    }

    @Override
    public void end(boolean interrupted) {
        // 3. Kill motor
        hood.stop();

        // 4. Zero encoder — block until confirmed so the default command's
        //    first holdPosition() call reads 0, not the pre-zero stale value
        hood.hoodMotor.setPosition(0.0, 0.050);

        // 5. Re-enable soft limits with lower bound at 0
        var limitConfigs = new SoftwareLimitSwitchConfigs();
        limitConfigs.withReverseSoftLimitEnable(true);
        limitConfigs.withReverseSoftLimitThreshold(0.0);
        limitConfigs.withForwardSoftLimitEnable(true);
        limitConfigs.withForwardSoftLimitThreshold(6.0);
        hood.hoodMotor.getConfigurator().apply(limitConfigs, 0.050);

        System.out.println(spikeLatched
            ? "[AmpZeroing] Spike detected — zeroed at hard stop. Soft limits restored."
            : "[AmpZeroing] Interrupted — motor stopped. Soft limits restored.");
    }
}