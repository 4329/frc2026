package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String swerveLimelight;
    private final String turretLimelight;

    private boolean hasTarget = false;
    private double targetTX = 0.0;
    private double targetTY = 0.0;
    private double targetArea = 0.0;
    private double targetDistance = 0.0;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.swerveLimelight = VisionConstants.LIMELIGHT_SWERVE_NAME;
        this.turretLimelight = VisionConstants.LIMELIGHT_TURRET_NAME;

        // Swerve limelight camera pose
        LimelightHelpers.setCameraPose_RobotSpace(
            swerveLimelight,
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getX(),
            -VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getY(),
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getZ(),
            Math.toDegrees(VisionConstants.ROBOT_TO_SWERVE_CAMERA.getRotation().getX()),
            Math.toDegrees(VisionConstants.ROBOT_TO_SWERVE_CAMERA.getRotation().getY()),
            Math.toDegrees(VisionConstants.ROBOT_TO_SWERVE_CAMERA.getRotation().getZ())
        );

        // Turret limelight camera pose
        LimelightHelpers.setCameraPose_RobotSpace(
            turretLimelight,
            VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getX(),
            -VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getY(),
            VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getZ(),
            Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getX()),
            Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getY()),
            Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getZ())
        );

        LimelightHelpers.setPipelineIndex(swerveLimelight, 0);
        LimelightHelpers.setPipelineIndex(turretLimelight, 1);

        LimelightHelpers.setLEDMode_PipelineControl(swerveLimelight);
        LimelightHelpers.setLEDMode_PipelineControl(turretLimelight);
    }

    @Override
    public void periodic() {
        var pigeon = drivetrain.getPigeon2();
        double yaw = pigeon.getYaw().getValueAsDouble();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();

        // Both limelights need orientation for MegaTag2
        LimelightHelpers.SetRobotOrientation(swerveLimelight, yaw, yawRate, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(turretLimelight, yaw, yawRate, 0, 0, 0, 0);

        // Target tracking using swerve limelight
        boolean tv = LimelightHelpers.getTV(swerveLimelight);
        double tid = LimelightHelpers.getFiducialID(swerveLimelight);

        if (tv && (int) tid == VisionConstants.TARGET_TAG_ID) {
            hasTarget = true;
            targetTX = LimelightHelpers.getTX(swerveLimelight);
            targetTY = LimelightHelpers.getTY(swerveLimelight);
            targetArea = LimelightHelpers.getTA(swerveLimelight);

            RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(swerveLimelight);
            targetDistance = 0.0;
            for (RawFiducial fiducial : rawFiducials) {
                if (fiducial.id == VisionConstants.TARGET_TAG_ID) {
                    targetDistance = fiducial.distToRobot;
                    break;
                }
            }

            if (targetDistance == 0.0 && targetArea > 0.1) {
                targetDistance = Math.sqrt(5.0 / targetArea);
            }
        } else {
            hasTarget = false;
            targetTX = 0.0;
            targetTY = 0.0;
            targetArea = 0.0;
            targetDistance = 0.0;
        }

        // Pose estimation from both limelights
        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false);

        processPoseEstimate(
            isRed ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(swerveLimelight)
                  : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(swerveLimelight),
            "Swerve"
        );

        processPoseEstimate(
            isRed ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(turretLimelight)
                  : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(turretLimelight),
            "Turret"
        );

        // Logging
        Logger.recordOutput("Vision/TargetFound", hasTarget);
        Logger.recordOutput("Vision/TargetTX", targetTX);
        Logger.recordOutput("Vision/TargetTY", targetTY);
        Logger.recordOutput("Vision/TargetArea", targetArea);
        Logger.recordOutput("Vision/TargetDistance", targetDistance);
    }

    private void processPoseEstimate(PoseEstimate poseEstimate, String cameraName) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) return;

        if (poseEstimate.tagCount >= 2) {
            drivetrain.addVisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                VisionConstants.MULTI_TAG_STD_DEVS
            );
        } else if (poseEstimate.tagCount == 1 && poseEstimate.avgTagDist < 4.0) {
            drivetrain.addVisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                VisionConstants.SINGLE_TAG_STD_DEVS
            );
        }

        Logger.recordOutput("Vision/" + cameraName + "/RobotPose", poseEstimate.pose);
        Logger.recordOutput("Vision/" + cameraName + "/TagCount", poseEstimate.tagCount);
        Logger.recordOutput("Vision/" + cameraName + "/AverageDist", poseEstimate.avgTagDist);
    }

    public boolean hasTarget() { return hasTarget; }
    public double getTargetTX() { return targetTX; }
    public double getTargetTY() { return targetTY; }
    public double getTargetArea() { return targetArea; }
    public double getTargetDistance() { return targetDistance; }

    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOff(turretLimelight);
    }

    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOn(turretLimelight);
    }
}