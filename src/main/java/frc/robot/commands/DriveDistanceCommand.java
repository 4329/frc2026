package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType; 
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveDistanceCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final double targetDistance;
    private final double speed;
    private final SwerveRequest.FieldCentric drive;

    private Pose2d starPose;


    public DriveDistanceCommand(CommandSwerveDrivetrain drivetrain, double targetDistance, double speed) {
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;
        this.speed = speed;

        this.drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        starPose = drivetrain.getState().Pose;
    }

    @Override
    public void execute() {
        drivetrain.setControl(drive.withVelocityX(speed).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double distanceTraveled = currentPose.getTranslation().getDistance(starPose.getTranslation());
        
        return distanceTraveled >= targetDistance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
