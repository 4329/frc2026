package frc.robot.commands.TurretCommands.HoodCommands;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;

public class HoodZeroCommand extends Command{
    private static final double CREEEP_DUTY_CYCLE = -0.5;
    private static final double CURRENT_SPIKE_AMPS = 40.0;

    private final HoodSubsystem hood;
    private final DutyCycleOut creepRequest = new DutyCycleOut(CREEEP_DUTY_CYCLE).withEnableFOC(true);

    private boolean spikeDetected = false;

    public HoodZeroCommand(HoodSubsystem hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    
    @Override
    public void initialize() {
        spikeDetected = false;
        disableReverseSoftLimit();
        hood.hoodMotor.setControl(creepRequest);
    }

    @Override
    public void execute() {
        System.out.println(hood.getSupplyCurrent());
        if (hood.getSupplyCurrent() >= CURRENT_SPIKE_AMPS || hood.getSupplyCurrent() <= -0.01) {
            spikeDetected = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return spikeDetected;
    }

    @Override
    public void end(boolean interrupted) {
        hood.hoodMotor.stopMotor();
        hood.setPosition(0.5);


        if (!interrupted) {
            hood.hoodMotor.setPosition(0.0);
            enableReverseSoftLimit();
        } else {
            enableReverseSoftLimit();
        }
    }

    private void disableReverseSoftLimit() {
        var cfg = new SoftwareLimitSwitchConfigs();
        hood.hoodMotor.getConfigurator().refresh(cfg);
        cfg.withReverseSoftLimitEnable(false);
        hood.hoodMotor.getConfigurator().apply(cfg);
    }

    private void enableReverseSoftLimit() {
        var cfg = new SoftwareLimitSwitchConfigs();
        hood.hoodMotor.getConfigurator().refresh(cfg);
        cfg.withReverseSoftLimitThreshold(0.0).withReverseSoftLimitEnable(true);
        hood.hoodMotor.getConfigurator().apply(cfg);
    }
}
