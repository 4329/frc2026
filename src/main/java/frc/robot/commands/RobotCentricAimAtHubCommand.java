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

public class RobotCentricAimAtHubCommand extends Command {

    private static final double MAX_TRANSLATION_SPEED = 1.0; // m/s
    private static final double AIM_TOLERANCE_DEG     = 1.0;

    private static final double kP = 5.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier          xSupplier;
    private final DoubleSupplier          ySupplier;

    private final PIDController rotationPID;

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotCentricAimAtHubCommand(
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
        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false);

        Translation2d hubCenter = isRed
            ? VisionConstants.RED_HUB_CENTER
            : VisionConstants.BLUE_HUB_CENTER;

        // Vector from robot to hub in field space
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        Translation2d toHub    = hubCenter.minus(robotPos);

        // Target yaw: robot's back faces the hub
        double hubFieldAngleDeg  = Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
        double targetRobotYawDeg = normalizeAngle(hubFieldAngleDeg + 180.0);

        double currentYawDeg = drivetrain.getState().Pose.getRotation().getDegrees();

        // Rotation PID output in rad/s
        double rotationRad = Math.toRadians(rotationPID.calculate(currentYawDeg, targetRobotYawDeg));

        // Joystick inputs are robot-relative in robot-centric mode — no frame rotation needed
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
        return false;
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