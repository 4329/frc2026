package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import frc.robot.commands.intakeCommands.SpinMotor44Command;
import frc.robot.commands.intakeCommands.IntakeGoToPositionCommand;
import frc.robot.commands.intakeCommands.SpinMotor13Command;
//import frc.robot.commands.intakeCommands.SpinCrashoutCommand;
import frc.robot.commands.intakeCommands.SpinDexer;
import frc.robot.subsystems.SpinMotor13thingy;
//import frc.robot.subsystems.SpinMotor44Subsystem;
import frc.robot.subsystems.SpinDexterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;

public class IntakeAndSpinCommandGroup extends ParallelCommandGroup {

    public IntakeAndSpinCommandGroup(
            IntakeSubsystem intake,
            SpinMotor13thingy crashout,
           // SpinMotor44Subsystem crashout44,
            NEO550ThroughTalonFXSSubsytem spinner,
            SpinDexterSubsystem spinDexter,
            double maxAngularRate) {

        

        addCommands(
            new IntakeGoToPositionCommand(intake, maxAngularRate, true),
            new SpinMotor13Command(crashout, 6.0, true),
           // new SpinMotor44Command(crashout44, 6.0, true),
           // new SpinCrashoutCommand(spinner, 6.0, true),
            new SpinDexer(spinDexter, 6.0)
        );
    }
}