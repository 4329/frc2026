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

public class AimAtPassCommand extends Command {

    private static final double MAX_TRANSLATION_SPEED = 1.0; // m/s
    private static final double AIM_TOLERANCE_DEG     = 1.0;
    private static final double FIELD_CENTRE_Y        = 4.0069; // half of 8.0137 m

    private static final double kP = 5.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier          xSupplier;
    private final DoubleSupplier          ySupplier;

    private final PIDController rotationPID;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AimAtPassCommand(
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

        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();

        // Pick whichever pass point is on the same side of the field as the robot
        boolean robotOnLeft = robotPos.getY() >= FIELD_CENTRE_Y;

        Translation2d passTarget;
        if (isRed) {
            passTarget = robotOnLeft ? VisionConstants.RED_PASS_LEFT
                                     : VisionConstants.RED_PASS_RIGHT;
        } else {
            passTarget = robotOnLeft ? VisionConstants.BLUE_PASS_LEFT
                                     : VisionConstants.BLUE_PASS_RIGHT;
        }

        Translation2d toTarget = passTarget.minus(robotPos);

        // +180° so the robot's back (turret side) faces the pass point
        double fieldAngleDeg   = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
        double targetYawDeg    = normalizeAngle(fieldAngleDeg + 180.0);
        double currentYawDeg   = drivetrain.getState().Pose.getRotation().getDegrees();

        double rotationRad = Math.toRadians(rotationPID.calculate(currentYawDeg, targetYawDeg));

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
        if (deg >  180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }
}