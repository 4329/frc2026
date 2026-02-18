package frc.robot.commands.TurretCommands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;


public class ShooterVolSpinCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double voltage;

    
    public ShooterVolSpinCommand(ShooterSubsystem shooter, double voltage) {
        this.shooter = shooter;
        this.voltage = voltage;
        addRequirements(shooter);
    }


    @Override
    public void execute() {
        shooter.spinVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    
}
