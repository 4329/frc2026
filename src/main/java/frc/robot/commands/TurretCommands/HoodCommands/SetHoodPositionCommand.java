package frc.robot.commands.TurretCommands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;

public class SetHoodPositionCommand extends Command {
    private final HoodSubsystem hood;
    private final double targetPosition;
    private static final double TOLERANCE = 0.2;

    public SetHoodPositionCommand(HoodSubsystem hood, double targetPosition) {
        this.hood = hood;
        this.targetPosition = targetPosition;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        System.out.println("---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        System.out.println(targetPosition);
        System.out.println(TOLERANCE);
        hood.setPosition(targetPosition);
    }

    

    @Override
    public void execute() {
        System.out.println(hood.getPosition());
    }

    @Override
    public boolean isFinished() {
        return hood.atPosition(targetPosition, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("YOUR SET HOOD POSITION COMMAND HAS ENDEDDDDDD AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        System.out.println("A");
        System.out.println("A");
        System.out.println("A");
        System.out.println("A");
        System.out.println("A");
        System.out.println("A");
        System.out.println("A");
        // hood.holdPosition();
    }
}