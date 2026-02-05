package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenMotorSubsystem;

public class PosKrakenMotorSpinCommand extends Command {
    private final KrakenMotorSubsystem motorYes;
    private final double targetPos;
    private static final double TOLERANCE = 0.1;


     public PosKrakenMotorSpinCommand(KrakenMotorSubsystem motorYes, double targetPos) {
        this.motorYes = motorYes;
        this.targetPos = targetPos;
        addRequirements(motorYes);
    }


    @Override
    public void execute() {
        motorYes.setPosition(targetPos);
        System.out.println(motorYes.getPosition());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(motorYes.getPosition() - targetPos) < TOLERANCE;
    }



    @Override
    public void end(boolean interrupted) {
    }
}


