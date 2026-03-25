package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurretCommands.HoodCommands.HoodZeroCommand;
import frc.robot.commands.TurretCommands.HoodCommands.SetHoodPositionCommand;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;

public class HoodToZeroCommandGroup extends SequentialCommandGroup {
    public HoodToZeroCommandGroup(HoodSubsystem hood) {
        addCommands(
            new SetHoodPositionCommand(hood, 0.5),
            new HoodZeroCommand(hood)
        );
    }
}
