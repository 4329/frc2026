package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;
import frc.robot.subsystems.SpinDexterSubsystem;

public class SpinDexer extends Command {
    private final SpinDexterSubsystem spinner;
    private final double voltage;

    
    public SpinDexer(SpinDexterSubsystem spinner, double voltage) {
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