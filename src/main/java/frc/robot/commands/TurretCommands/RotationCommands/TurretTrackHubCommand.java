package frc.robot.commands.TurretCommands.RotationCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;

public class TurretTrackHubCommand extends Command {
    private static final double GEAR_RATIO = 45.0;
    private static final double MAX_MOTOR_ROT = 12.5;
    private static final double MIN_MOTOR_ROT = -33.75;
    private static final double TURRET_ZERO_ANGLE_DEGREES = 180.0;
    private static final double POSITION_TOLERANCE_MOTOR_ROT = 0.5;
    private static final double SNAP_THRESHOLD = 1.0;

    private final RotateSubsystem turret;
    private final CommandSwerveDrivetrain drivetrain;

    public TurretTrackHubCommand(RotateSubsystem turret, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Translation2d hubCenter = DriverStation.getAlliance().map(a -> a == Alliance.Red ? VisionConstants.RED_HUB_CENTER : VisionConstants.BLUE_HUB_CENTER).orElse(VisionConstants.BLUE_HUB_CENTER);

        Translation2d robotPose = drivetrain.getState().Pose.getTranslation();
        Translation2d toHub = hubCenter.minus(robotPose);

        double hubFieldAngleDeg = Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));

        double robotYawDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        double hubRobotAngleDeg = hubFieldAngleDeg - robotYawDeg;

        hubRobotAngleDeg = normalizeAngle(hubRobotAngleDeg);

        double turretAngleDeg = normalizeAngle(hubRobotAngleDeg - TURRET_ZERO_ANGLE_DEGREES);

        double turretMotorRot = (-turretAngleDeg / 360.0) * GEAR_RATIO;

        double commandedMotorRot = clampWithSnap(turretMotorRot);

        turret.setPositionWithVelocity(commandedMotorRot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }


    private double normalizeAngle(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    private double clampWithSnap(double desiredMotorRot) {
        if (desiredMotorRot >= MIN_MOTOR_ROT && desiredMotorRot <= MAX_MOTOR_ROT) {
            if(desiredMotorRot > MAX_MOTOR_ROT - SNAP_THRESHOLD) {
                double alternate = desiredMotorRot - GEAR_RATIO;
                if (alternate >= MIN_MOTOR_ROT) return alternate;
            }
            if (desiredMotorRot < MIN_MOTOR_ROT + SNAP_THRESHOLD) {
                double alternate = desiredMotorRot + GEAR_RATIO;
                if (alternate <= MAX_MOTOR_ROT) return alternate;
            }
            return desiredMotorRot;
        }

        double alternate = desiredMotorRot > MAX_MOTOR_ROT
        ? desiredMotorRot - GEAR_RATIO
        : desiredMotorRot + GEAR_RATIO;

        if (alternate >= MIN_MOTOR_ROT && alternate <= MAX_MOTOR_ROT) {
            return alternate;
        }

        return Math.max(MIN_MOTOR_ROT, Math.min(MAX_MOTOR_ROT, desiredMotorRot));
    }
}
