package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.model.VisionLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;

public class VisionSubsystem extends SubsystemBase implements LoggedSubsystem {

    private final VisionLogAutoLogged visionLog = new VisionLogAutoLogged();

    private final CommandSwerveDrivetrain drivetrain;
    private final RotateSubsystem turret;
    private final String swerveLimelight;
    private final String turretLimelight;

    private boolean hasLimelightTarget = false;
    private boolean usingPoseFallback  = false;
    private double  targetTX           = 0.0;
    private double  targetTY           = 0.0;
    private double  targetArea         = 0.0;
    private double  limelightDistance  = 0.0;
    private double  poseDistance       = 0.0;
    private double  targetDistance     = 0.0;
    private int     visibleHubTags     = 0;

    private boolean hasInitializedPose    = true;
    private boolean poseEstimationEnabled = true;


    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, RotateSubsystem turret) {
        this.drivetrain      = drivetrain;
        this.turret          = turret;
        this.swerveLimelight = VisionConstants.LIMELIGHT_SWERVE_NAME;
        this.turretLimelight = VisionConstants.LIMELIGHT_TURRET_NAME;

        LimelightHelpers.setCameraPose_RobotSpace(
            swerveLimelight,
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getX(),
            -VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getY(),
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getZ(),
            180.0,
            20.0,
            180.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            turretLimelight,
            VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getX(),
            -VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getY(),
            VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getZ(),
            0,
            -20,
            180
        );

        LimelightHelpers.setPipelineIndex(swerveLimelight, 0);
        LimelightHelpers.setPipelineIndex(turretLimelight, 0);

        LimelightHelpers.setLEDMode_PipelineControl(swerveLimelight);
        LimelightHelpers.setLEDMode_PipelineControl(turretLimelight);
    }

    @Override
    public LoggableInputs log() {
        visionLog.hasLimelightTarget = hasLimelightTarget;
        visionLog.usingPoseFallback  = usingPoseFallback;
        visionLog.hasInitializedPose = hasInitializedPose;
        visionLog.targetTX           = targetTX;
        visionLog.targetTY           = targetTY;
        visionLog.targetArea         = targetArea;
        visionLog.limelightDistance  = limelightDistance;
        visionLog.poseDistance       = poseDistance;
        visionLog.targetDistance     = targetDistance;
        visionLog.visibleHubTags     = visibleHubTags;
        return visionLog;
    }

    @Override
    public String getNameLog() {
        return "Vision";
    }

    @Override
    public void periodic() {
        double headingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        boolean isRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);

        LimelightHelpers.SetRobotOrientation(swerveLimelight, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(turretLimelight, headingDeg, 0, 0, 0, 0, 0);

        // updateTurretCameraPose();

        processPoseEstimate(
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(swerveLimelight),
            swerveLimelight
        );
        processPoseEstimate(
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(turretLimelight),
            turretLimelight
        );

        int[] hubTagIds = isRed
            ? VisionConstants.RED_HUB_TAG_IDS
            : VisionConstants.BLUE_HUB_TAG_IDS;

        Translation2d hubCenter = isRed
            ? VisionConstants.RED_HUB_CENTER
            : VisionConstants.BLUE_HUB_CENTER;

        updateTurretTargeting(hubCenter, hubTagIds);

        // AdvantageKit
        Logger.recordOutput("Vision/HasLimelightTarget", hasLimelightTarget);
        Logger.recordOutput("Vision/UsingPoseFallback",  usingPoseFallback);
        Logger.recordOutput("Vision/LimelightDistance",  limelightDistance);
        Logger.recordOutput("Vision/PoseDistance",       poseDistance);
        Logger.recordOutput("Vision/TargetDistance",     targetDistance);
        Logger.recordOutput("Vision/VisibleHubTags",     visibleHubTags);
        Logger.recordOutput("Vision/TargetTX",           targetTX);
        Logger.recordOutput("Vision/TargetTY",           targetTY);
        Logger.recordOutput("Vision/HubCenter",          hubCenter);

        // SmartDashboard
        SmartDashboard.putBoolean("Vision/Hub Target Locked",     hasLimelightTarget);
        SmartDashboard.putBoolean("Vision/Using Pose Fallback",   usingPoseFallback);
        SmartDashboard.putNumber("Vision/Hub Distance (m)",       targetDistance);
        SmartDashboard.putNumber("Vision/Limelight Distance (m)", limelightDistance);
        SmartDashboard.putNumber("Vision/Pose Distance (m)",      poseDistance);
        SmartDashboard.putNumber("Vision/Hub Tags Visible",       visibleHubTags);
        SmartDashboard.putNumber("Vision/Target TX",              targetTX);
        SmartDashboard.putNumber("Vision/Target TY",              targetTY);
        SmartDashboard.putString("Vision/Alliance",               isRed ? "RED" : "BLUE");
    }

    // private void updateTurretCameraPose() {
    //     double turretAngleRad = turret.getPosition() * 2.0 * Math.PI;

    //     double camOffsetX = VisionConstants.TURRET_AXIS_X
    //         + VisionConstants.TURRET_CAMERA_RADIUS * Math.cos(turretAngleRad);
    //     double camOffsetY = VisionConstants.TURRET_AXIS_Y
    //         + VisionConstants.TURRET_CAMERA_RADIUS * Math.sin(turretAngleRad);

    //     double camZ     = VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getZ();
    //     double camPitch = Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getY());
    //     double camRoll  = Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getX());
    //     double camYaw   = Math.toDegrees(turretAngleRad) + 180.0;

    //     LimelightHelpers.setCameraPose_RobotSpace(
    //         turretLimelight,
    //         camOffsetX,
    //         -camOffsetY,
    //         camZ,
    //         camRoll,
    //         camPitch,
    //         camYaw
    //     );
    // }

    private void updateTurretTargeting(Translation2d hubCenter, int[] hubTagIds) {
        var robotPose = drivetrain.getState().Pose;
        double turretAngleRad = turret.getPosition() * 2.0 * Math.PI;

        double camX = VisionConstants.TURRET_AXIS_X
            + VisionConstants.TURRET_CAMERA_RADIUS * Math.cos(turretAngleRad);
        double camY = VisionConstants.TURRET_AXIS_Y
            + VisionConstants.TURRET_CAMERA_RADIUS * Math.sin(turretAngleRad);

        Translation2d turretCameraField = robotPose.transformBy(
            new Transform2d(
                new Translation2d(camX, camY),
                new Rotation2d(turretAngleRad)
            )
        ).getTranslation();

        poseDistance = turretCameraField.getDistance(hubCenter);

        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(turretLimelight);

        double distSum = 0.0;
        double txSum   = 0.0;
        double tySum   = 0.0;
        double areaSum = 0.0;
        int    found   = 0;

        for (RawFiducial f : fiducials) {
            if (isHubTag(f.id, hubTagIds)) {
                distSum += f.distToRobot;
                txSum   += f.txnc;
                tySum   += f.tync;
                areaSum += f.ta;
                found++;
            }
        }

        visibleHubTags = found;

        if (found > 0) {
            limelightDistance  = (distSum / found) + 0.6;
            targetTX           = txSum   / found;
            targetTY           = tySum   / found;
            targetArea         = areaSum / found;
            targetDistance     = limelightDistance;
            hasLimelightTarget = true;
            usingPoseFallback  = false;
        } else {
            limelightDistance  = 0.0;
            targetTX           = 0.0;
            targetTY           = 0.0;
            targetArea         = 0.0;
            targetDistance     = poseDistance;
            hasLimelightTarget = false;
            usingPoseFallback  = true;
        }
    }

    private boolean isHubTag(int id, int[] hubTagIds) {
        for (int hubId : hubTagIds) {
            if (id == hubId) return true;
        }
        return false;
    }

    private void processPoseEstimate(PoseEstimate estimate, String cameraName) {
        if (!poseEstimationEnabled) return;
        if (estimate == null || estimate.tagCount == 0) return;

        if (!hasInitializedPose && estimate.tagCount >= 2) {
            final int SAMPLE_COUNT = 10;
            double[] headingSamples = new double[SAMPLE_COUNT];
            double[] txSamples     = new double[SAMPLE_COUNT];
            double[] tySamples     = new double[SAMPLE_COUNT];
            int validSamples = 0;

            for (int i = 0; i < SAMPLE_COUNT; i++) {
                PoseEstimate s = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
                if (s != null && s.tagCount >= 2) {
                    headingSamples[validSamples] = s.pose.getRotation().getDegrees();
                    txSamples[validSamples]      = s.pose.getX();
                    tySamples[validSamples]      = s.pose.getY();
                    validSamples++;
                }
            }

            if (validSamples < 6) return;

            // Find the largest heading cluster to resolve MT1 ambiguity
            int bestIdx   = 0;
            int bestCount = 0;
            for (int i = 0; i < validSamples; i++) {
                int count = 0;
                for (int j = 0; j < validSamples; j++) {
                    if (i == j) continue;
                    double diff = Math.abs(headingSamples[i] - headingSamples[j]);
                    if (diff > 180.0) diff = 360.0 - diff;
                    if (diff < 20.0) count++;
                }
                if (count > bestCount) {
                    bestCount = count;
                    bestIdx   = i;
                }
            }

            // Abort if samples are too scattered — robot may still be moving
            if (bestCount < 3) return;

            // Average the clustered samples
            double headingSum = 0;
            double txSum      = 0;
            double tySum      = 0;
            int    clusterN   = 0;
            for (int i = 0; i < validSamples; i++) {
                double diff = Math.abs(headingSamples[i] - headingSamples[bestIdx]);
                if (diff > 180.0) diff = 360.0 - diff;
                if (diff < 20.0) {
                    headingSum += headingSamples[i];
                    txSum      += txSamples[i];
                    tySum      += tySamples[i];
                    clusterN++;
                }
            }

            double recoveredHeading = headingSum / clusterN;
            double recoveredX       = txSum      / clusterN;
            double recoveredY       = tySum      / clusterN;

            Pose2d resetPose = new Pose2d(recoveredX, recoveredY, Rotation2d.fromDegrees(recoveredHeading));

            LimelightHelpers.SetRobotOrientation(swerveLimelight, recoveredHeading, 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(turretLimelight, recoveredHeading, 0, 0, 0, 0, 0);
            drivetrain.resetPose(resetPose);
            drivetrain.getPigeon2().setYaw(recoveredHeading);

            PoseEstimate seededMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
            if (seededMT2 != null && seededMT2.tagCount >= 2) {
                drivetrain.addVisionMeasurement(
                    seededMT2.pose, seededMT2.timestampSeconds, VisionConstants.MULTI_TAG_STD_DEVS);
            }

            hasInitializedPose = true;
            Logger.recordOutput("Vision/PoseInitialized", true);
            return;
        }

        if (estimate.tagCount >= 2) {
            drivetrain.addVisionMeasurement(
                estimate.pose, estimate.timestampSeconds, VisionConstants.MULTI_TAG_STD_DEVS);
        } else if (estimate.tagCount == 1 && estimate.avgTagDist < 4.0) {
            drivetrain.addVisionMeasurement(
                estimate.pose, estimate.timestampSeconds, VisionConstants.SINGLE_TAG_STD_DEVS);
        }
    }

    public void resetPoseInitialization() {
        hasInitializedPose = false;
    }

    public void enablePoseEstimation() {
        poseEstimationEnabled = true;
    }

    public void disablePoseEstimation() {
        poseEstimationEnabled = false;
    }

    // --- Public getters ---
    public boolean hasTarget()               { return hasLimelightTarget || usingPoseFallback; }
    public boolean hasLimelightTarget()      { return hasLimelightTarget; }
    public boolean isUsingPoseFallback()     { return usingPoseFallback; }
    public boolean isPoseEstimationEnabled() { return poseEstimationEnabled; }
    public double  getTargetTX()             { return targetTX; }
    public double  getTargetTY()             { return targetTY; }
    public double  getTargetArea()           { return targetArea; }
    public double  getTargetDistance()       { return targetDistance; }
    public double  getLimelightDistance()    { return limelightDistance; }
    public double  getPoseDistance()         { return poseDistance; }
    public int     getVisibleHubTags()       { return visibleHubTags; }

    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOff(turretLimelight);
    }

    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOn(turretLimelight);
    }
}