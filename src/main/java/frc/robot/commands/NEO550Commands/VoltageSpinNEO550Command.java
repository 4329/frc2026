
package frc.robot.commands.NEO550Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;

public class VoltageSpinNEO550Command extends Command {
    private final NEO550ThroughTalonFXSSubsytem spinner13;
    private final double voltage;

    
    public VoltageSpinNEO550Command(NEO550ThroughTalonFXSSubsytem spinner, double voltage) {
        this.spinner13 = spinner;
        this.voltage = voltage;
        addRequirements(spinner);
    }


    @Override
    public void execute() {
        spinner13.spinVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        spinner13.stop();
    }
    
}
