package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenMotorSubsystem;

public class VolKrakenMotorSpinCommand extends Command {
    private final KrakenMotorSubsystem motorYes;
    private final double voltage;

    
    public VolKrakenMotorSpinCommand(KrakenMotorSubsystem motorYes, double voltage) {
        this.motorYes = motorYes;
        this.voltage = voltage;
        addRequirements(motorYes);
    }


    @Override
    public void execute() {
        motorYes.spinVoltage(voltage);
        System.out.println(motorYes.getPosition());
    }

    @Override
    public void end(boolean interrupted) {
        motorYes.stop();
    }
    
}


