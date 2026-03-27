package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;

public class DoAFunctionalCommand extends SequentialCommandGroup {

    private static final double DRIVE_SPEED = 1.0;  // m/s
    private static final double ROT_SPEED   = 2.0;  // rad/s

    public DoAFunctionalCommand(
            CommandSwerveDrivetrain drivetrain,
            XboxController controller,
            IntakePivotSubsystem intakePivot,
            IntakeSpinSubsystem intakeSpin,
            SpindexterSubsystem spindexter,
            KickerSubsystem kicker,
            HoodSubsystem hood,
            ShooterSubsystem shooter) {

        addCommands(
            Commands.waitUntil(controller::getAButtonPressed),

            Commands.run(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
                    .withVelocityX(DRIVE_SPEED)), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.run(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
                    .withVelocityX(-DRIVE_SPEED)), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.run(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
                    .withVelocityY(-DRIVE_SPEED)), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.run(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
                    .withVelocityY(DRIVE_SPEED)), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.run(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
                    .withRotationalRate(-ROT_SPEED)), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.run(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
                    .withRotationalRate(ROT_SPEED)), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.runOnce(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.Idle()), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.runOnce(() -> drivetrain.setControl(
                new com.ctre.phoenix6.swerve.SwerveRequest.Idle()), drivetrain)
                .until(controller::getAButtonPressed),

            Commands.runOnce(() -> intakePivot.setPosition(6.4)).until(controller::getAButtonPressed),

            Commands.runOnce(() -> intakeSpin.setVelocity(60)).until(controller::getAButtonPressed),

            Commands.runOnce(() -> intakeSpin.stop()).until(controller::getAButtonPressed),

            Commands.runOnce(() -> intakePivot.setPosition(0.0)).until(controller::getAButtonPressed)
        );
    }
}