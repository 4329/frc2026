package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeCommands.IntakePivotCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;

public class IntakeSubsystemCommandGroup extends ParallelCommandGroup {
    public IntakeSubsystemCommandGroup(IntakePivotSubsystem pivot, IntakeSpinSubsystem spin) {
        addCommands(
            new IntakePivotCommand(pivot, 5.9),
            new IntakeSpinCommand(spin, 60));
          Commands.run(() -> 
                 System.out.println("Intake Pivot Rotation: " + pivot.getPosition() + 
                                  " | Spin RPS: " + spin.getVelocity())
        );
        
    }

    
}