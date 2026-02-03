package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;

public class VoltageSpinNEO550Command extends Command {
    private final NEO550ThroughTalonFXSSubsytem spinner;
    private final double voltage;

    
    public VoltageSpinNEO550Command(NEO550ThroughTalonFXSSubsytem spinner, double voltage) {
        this.spinner = spinner;
        this.voltage = voltage;
        addRequirements(spinner);
    }


    @Override
    public void execute() {
        spinner.spinVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        spinner.stop();
    }
    
}
