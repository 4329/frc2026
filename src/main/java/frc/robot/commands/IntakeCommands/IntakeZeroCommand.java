package frc.robot.commands.IntakeCommands;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;

public class IntakeZeroCommand extends Command{
    private static final double CREEEP_DUTY_CYCLE = -0.10;
    private static final double CURRENT_SPIKE_AMPS = 4.0;

    private final IntakePivotSubsystem pivot;
    private final DutyCycleOut creepRequest = new DutyCycleOut(CREEEP_DUTY_CYCLE).withEnableFOC(true);

    private boolean spikeDetected = false;

    public IntakeZeroCommand(IntakePivotSubsystem pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    
    @Override
    public void initialize() {
        spikeDetected = false;
        disableReverseSoftLimit();
        pivot.pivotMotor.setControl(creepRequest);
    }

    @Override
    public void execute() {
        System.out.println(pivot.getStatorCurrent());
        if (pivot.getStatorCurrent() >= CURRENT_SPIKE_AMPS || pivot.getStatorCurrent() <= -0.01) {
            spikeDetected = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return spikeDetected;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.pivotMotor.stopMotor();


        if (!interrupted) {
            pivot.pivotMotor.setPosition(0.0);
            pivot.setPosition(0.0);
            enableReverseSoftLimit();
        } else {
            enableReverseSoftLimit();
        }
    }

    private void disableReverseSoftLimit() {
        var cfg = new SoftwareLimitSwitchConfigs();
        pivot.pivotMotor.getConfigurator().refresh(cfg);
        cfg.withReverseSoftLimitEnable(false);
        pivot.pivotMotor.getConfigurator().apply(cfg);
    }

    private void enableReverseSoftLimit() {
        var cfg = new SoftwareLimitSwitchConfigs();
        pivot.pivotMotor.getConfigurator().refresh(cfg);
        cfg.withReverseSoftLimitThreshold(0.0).withReverseSoftLimitEnable(true);
        pivot.pivotMotor.getConfigurator().apply(cfg);
    }
}
