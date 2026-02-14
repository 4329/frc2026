package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretMotorSubsystem;


public class TurretRotationsPerSecCommand extends Command {
    private final TurretMotorSubsystem turretMotor;
    private final double rotationsPerSecond;

    
    public TurretRotationsPerSecCommand(TurretMotorSubsystem turretMotor, double rotationsPerSecond) {
        this.turretMotor = turretMotor;
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(turretMotor);
    }


    @Override
    public void execute() {
        turretMotor.setVelocity(rotationsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        turretMotor.stop();
    }
    
}
