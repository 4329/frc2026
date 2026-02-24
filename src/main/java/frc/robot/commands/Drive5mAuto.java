package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive5mAuto {

    public static Command create(CommandSwerveDrivetrain drivetrain) {
        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.Idle idle = new SwerveRequest.Idle();

        return Commands.sequence(Commands.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero), drivetrain), new DriveDistanceCommand(drivetrain, 5.0, 0.5), drivetrain.applyRequest(() -> idle));
    }
}
