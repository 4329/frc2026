package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSpinSubsystem;

public class IntakeSubsystemOnTrueCommandGroup extends ParallelCommandGroup {
    public IntakeSubsystemOnTrueCommandGroup(IntakePivotSubsystem pivot, IntakeSpinSubsystem spin) {
        addCommands(
            new IntakePivotCommand(pivot, 4),
            new IntakeSpinCommand(spin, 0.45));
          Commands.run(() -> 
                 System.out.println("Intake Pivot Rotation: " + pivot.getPosition() + 
                                  " | Spin RPS: " + spin.getVelocity())
        );
        
    }

    
}