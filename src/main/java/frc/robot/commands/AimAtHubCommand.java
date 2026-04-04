package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AimAtHubCommand extends Command {

    private static final double MAX_TRANSLATION_SPEED = 1.0; // m/s
    private static final double AIM_TOLERANCE_DEG     = 1.0;

    // PID for rotation — tune kP to taste
    private static final double kP = 5.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier          xSupplier;
    private final DoubleSupplier          ySupplier;

    private final PIDController rotationPID;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AimAtHubCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier  = xSupplier;
        this.ySupplier  = ySupplier;

        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.enableContinuousInput(-180.0, 180.0);
        rotationPID.setTolerance(AIM_TOLERANCE_DEG);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
    }

    @Override
    public void execute() {
        // Get hub center for current alliance — this is the geometric center of the
        // hub structure, not the april tag positions, so aiming is accurate from any angle
        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false);

        Translation2d hubCenter = isRed
            ? VisionConstants.RED_HUB_CENTER
            : VisionConstants.BLUE_HUB_CENTER;

        // Vector from robot center to hub center in field space
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        Translation2d toHub    = hubCenter.minus(robotPos);

        // Angle the robot needs to face so the turret (facing backward) points at hub center
        // Add 180° so the robot's back faces the hub instead of its front
        double hubFieldAngleDeg  = Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
        double targetRobotYawDeg = normalizeAngle(hubFieldAngleDeg + 180.0);

        // Current robot yaw from pose estimator
        double currentYawDeg = drivetrain.getState().Pose.getRotation().getDegrees();

        // PID output in degrees/s, converted to rad/s for swerve request
        double rotationRad = Math.toRadians(rotationPID.calculate(currentYawDeg, targetRobotYawDeg));

        // Clamp translation to reduced speed, driver keeps full X/Y control
        double xVel = xSupplier.getAsDouble() * MAX_TRANSLATION_SPEED;
        double yVel = ySupplier.getAsDouble() * MAX_TRANSLATION_SPEED;

        drivetrain.setControl(
            driveRequest
                .withVelocityX(-yVel)
                .withVelocityY(-xVel)
                .withRotationalRate(rotationRad)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button released
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    private double normalizeAngle(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0)  deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }
}