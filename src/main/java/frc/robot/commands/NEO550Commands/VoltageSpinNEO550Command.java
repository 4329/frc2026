<<<<<<<< HEAD:src/main/java/frc/robot/commands/intakeCommands/VoltageSpinNEO550Command.java
package frc.robot.commands.intakeCommands;
========
package frc.robot.commands.NEO550Commands;
>>>>>>>> origin/main:src/main/java/frc/robot/commands/NEO550Commands/VoltageSpinNEO550Command.java

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
