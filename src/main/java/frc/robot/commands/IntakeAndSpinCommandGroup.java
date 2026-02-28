package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intakeCommands.IntakeGoToPositionCommand;
import frc.robot.commands.intakeCommands.KickerCommand;
import frc.robot.commands.intakeCommands.SpinMotor13IndefinitelyCommand;
import frc.robot.commands.intakeCommands.SpinDexer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpinMotor13Subsystem;
import frc.robot.subsystems.SpinDexterSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class IntakeAndSpinCommandGroup extends ParallelCommandGroup {

    public IntakeAndSpinCommandGroup(
            IntakeSubsystem intake,
            SpinMotor13Subsystem spin13,
            KickerSubsystem kicker,
            SpinDexterSubsystem spinDexter,
            double maxAngularRate) {

        addCommands(
            new IntakeGoToPositionCommand(intake, maxAngularRate, true),
            new SpinMotor13IndefinitelyCommand(spin13, 6.0, true),
            new SpinDexer(spinDexter, 6.0),
            new KickerCommand(kicker, 6.0, true) // uses the correct KickerSubsystem
        );
    }
}