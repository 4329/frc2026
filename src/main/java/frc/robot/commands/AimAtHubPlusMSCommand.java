package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AimAtHubPlusMSCommand extends Command {

    private static final double MAX_TRANSLATION_SPEED = 1.5;
    private static final double AIM_TOLERANCE_DEG     = 1.0;

    /**
     * Approximate fuel speed leaving the shooter in m/s.
     * Tune empirically — same constant as TurretTrackHubCommand.
     */
    private static final double FUEL_SPEED_MS = 25.0;

    /** Number of iterative refinement passes for the compensation vector. */
    private static final int COMPENSATION_ITERATIONS = 3;

    private static final double kP = 5.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier          xSupplier;
    private final DoubleSupplier          ySupplier;

    private final PIDController rotationPID;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AimAtHubPlusMSCommand(
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

        Translation2d robotPos     = drivetrain.getState().Pose.getTranslation();
        ChassisSpeeds speeds       = drivetrain.getState().Speeds;

        Translation2d virtualTarget = hubCenter;
        double flightTime           = 0.0;

        for (int i = 0; i < COMPENSATION_ITERATIONS; i++) {
            double dist = robotPos.getDistance(virtualTarget);
            flightTime  = dist / FUEL_SPEED_MS;

            virtualTarget = hubCenter.minus(
                new Translation2d(
                    speeds.vxMetersPerSecond * flightTime,
                    speeds.vyMetersPerSecond * flightTime
                )
            );
        }

        // Aim robot so its back faces the compensated virtual target
        Translation2d toTarget   = virtualTarget.minus(robotPos);
        double hubFieldAngleDeg  = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
        double targetRobotYawDeg = normalizeAngle(hubFieldAngleDeg + 180.0);
        double currentYawDeg     = drivetrain.getState().Pose.getRotation().getDegrees();

        double rotationRad = Math.toRadians(rotationPID.calculate(currentYawDeg, targetRobotYawDeg));

        double xVel = xSupplier.getAsDouble() * MAX_TRANSLATION_SPEED;
        double yVel = ySupplier.getAsDouble() * MAX_TRANSLATION_SPEED;

        drivetrain.setControl(
            driveRequest
                .withVelocityX(-yVel)
                .withVelocityY(-xVel)
                .withRotationalRate(rotationRad)
        );

        // Log for tuning
        Logger.recordOutput("AimAtHub/VirtualTargetX",   virtualTarget.getX());
        Logger.recordOutput("AimAtHub/VirtualTargetY",   virtualTarget.getY());
        Logger.recordOutput("AimAtHub/FlightTimeSec",    flightTime);
        Logger.recordOutput("AimAtHub/CompensationDX",   hubCenter.getX() - virtualTarget.getX());
        Logger.recordOutput("AimAtHub/CompensationDY",   hubCenter.getY() - virtualTarget.getY());
        Logger.recordOutput("AimAtHub/TargetYawDeg",     targetRobotYawDeg);
        Logger.recordOutput("AimAtHub/CurrentYawDeg",    currentYawDeg);
        Logger.recordOutput("AimAtHub/YawErrorDeg",      targetRobotYawDeg - currentYawDeg);
        Logger.recordOutput("AimAtHub/RobotVX",          speeds.vxMetersPerSecond);
        Logger.recordOutput("AimAtHub/RobotVY",          speeds.vyMetersPerSecond);
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