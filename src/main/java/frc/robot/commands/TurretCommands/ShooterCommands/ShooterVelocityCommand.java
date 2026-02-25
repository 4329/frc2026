package frc.robot.commands.TurretCommands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;

public class ShooterVelocityCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double rotationsPerSecond;

    
    public ShooterVelocityCommand(ShooterSubsystem shooter, double rotationsPerSecond) {
        this.shooter = shooter;
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(shooter);
    }


    @Override
    public void execute() {
        shooter.setVelocity(rotationsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    
}
