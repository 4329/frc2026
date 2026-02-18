package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldCentricSupplier;
    private final double maxSpeed;
    private final double maxAngularRate;

    private final SwerveRequest.FieldCentric fieldCentricRequest;
    private final SwerveRequest.RobotCentric robotCentricRequest;

    public DriveCommand(CommandSwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, BooleanSupplier fieldCentricSupplier, double maxSpeed, double maxAngularRate) {

        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        this.fieldCentricRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        this.robotCentricRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xSpeed = -ySupplier.getAsDouble() * maxSpeed;
        double ySpeed = -xSupplier.getAsDouble() * maxSpeed;
        double rotSpeed = -rotationSupplier.getAsDouble() * maxAngularRate;

        if (fieldCentricSupplier.getAsBoolean()) {
            drivetrain.setControl(
                fieldCentricRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotSpeed)
            );
        } else {
            drivetrain.setControl(
                robotCentricRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotSpeed)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false; // This is a default command, so it never finishes
    }
}
