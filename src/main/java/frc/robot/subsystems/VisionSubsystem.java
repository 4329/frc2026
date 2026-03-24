package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

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
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final RotateSubsystem turret;
    private final String swerveLimelight;
    private final String turretLimelight;
    
    private boolean hasLimelightTarget = false;
    private boolean usingPoseFallback = false;
    private double targetTX = 0.0;
    private double targetTY = 0.0;
    private double targetArea = 0.0;
    private double limelightDistance = 0.0;
    private double poseDistance = 0.0;
    private double targetDistance = 0.0;
    private int visibleHubTags = 0;
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, RotateSubsystem turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.swerveLimelight = VisionConstants.LIMELIGHT_SWERVE_NAME;
        this.turretLimelight = VisionConstants.LIMELIGHT_TURRET_NAME;
        
        LimelightHelpers.setCameraPose_RobotSpace(
            swerveLimelight,
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getX(),  // Forward
            -VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getY(), // Left (negate Y)
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getZ(),  // Up
            Math.toDegrees(VisionConstants.ROBOT_TO_SWERVE_CAMERA.getRotation().getX()), // Roll
            Math.toDegrees(VisionConstants.ROBOT_TO_SWERVE_CAMERA.getRotation().getY()), // Pitch
            Math.toDegrees(VisionConstants.ROBOT_TO_SWERVE_CAMERA.getRotation().getZ())  // Yaw
        );

        LimelightHelpers.setPipelineIndex(swerveLimelight, 0);
        LimelightHelpers.setPipelineIndex(turretLimelight, 0);
        
        LimelightHelpers.setLEDMode_PipelineControl(swerveLimelight);
        LimelightHelpers.setLEDMode_PipelineControl(turretLimelight);
    }
    
    @Override
    public void periodic() {
        var pigeon = drivetrain.getPigeon2();
        double yaw = pigeon.getYaw().getValueAsDouble();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        
        LimelightHelpers.SetRobotOrientation(swerveLimelight, yaw, yawRate, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(turretLimelight, yaw, yawRate, 0, 0, 0, 0);

        updateTurretCameraPose();
        
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


        Translation2d hubCenter = isRed
            ? VisionConstants.RED_HUB_CENTER
            : VisionConstants.BLUE_HUB_CENTER;

        int[] hubTagIds = isRed
            ? VisionConstants.RED_HUB_TAG_IDS
            : VisionConstants.BLUE_HUB_TAG_IDS;


        updateTurretTrageting(hubCenter, hubTagIds);
        
        Logger.recordOutput("Vision/HasLimelightTarget", hasLimelightTarget);
        Logger.recordOutput("Vision/TargetFresh", usingPoseFallback);
        Logger.recordOutput("Vision/LimelightDistance", limelightDistance);
        Logger.recordOutput("Vision/PoseDistance", poseDistance);
        Logger.recordOutput("Vision/TargetDistance", targetDistance);
        Logger.recordOutput("Vision/VisibleHubTags", visibleHubTags);
        Logger.recordOutput("Vision/TargetTX", targetTX);
        Logger.recordOutput("Vision/TargetTY", targetTY);
        Logger.recordOutput("Vision/HubCenter", hubCenter);        

        SmartDashboard.putBoolean("Vision/Hub Target Locked", hasLimelightTarget);
        SmartDashboard.putBoolean("Vision/Using Pose Fallback", usingPoseFallback);
        SmartDashboard.putNumber("Vision/Hub Distance (m)", targetDistance);
        SmartDashboard.putNumber("Vision/Limelight Distance (m)", limelightDistance);
        SmartDashboard.putNumber("Vision/Pose Distance (m)", poseDistance);
        SmartDashboard.putNumber("Vision/Hub Tags Visible", visibleHubTags);
        SmartDashboard.putNumber("Vision/Target TX", targetTX);
        SmartDashboard.putNumber("Vision/Target TY", targetTY);
        SmartDashboard.putString("Vision/Alliance", isRed ? "RED" : "BLUE");
    }

    private void updateTurretCameraPose() {
        double turretAngleRad = turret.getPosition() * 2.0 * Math.PI;

        double camOffsetX = VisionConstants.TURRET_AXIS_X + VisionConstants.TURRET_CAMERA_RADIUS * Math.cos(turretAngleRad);
        double camOffsetY = VisionConstants.TURRET_AXIS_Y + VisionConstants.TURRET_CAMERA_RADIUS * Math.sin(turretAngleRad);

        double camZ = VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getZ();
        double camPitch = Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getY());
        double camRoll = Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getX());

        double camYaw = Math.toDegrees(turretAngleRad) + 180.0;

        LimelightHelpers.setCameraPose_RobotSpace(turretLimelight, camOffsetX, -camOffsetY, camZ, camRoll, camPitch, camYaw);
    }

    private void updateTurretTrageting(Translation2d hubCenter, int[] hubTagIds) {
        var robotPose = drivetrain.getState().Pose;
        double turretAngleRad = turret.getPosition() * 2.0 * Math.PI;

        double camX = VisionConstants.TURRET_AXIS_X + VisionConstants.TURRET_CAMERA_RADIUS * Math.cos(turretAngleRad);
        double camY = VisionConstants.TURRET_AXIS_Y + VisionConstants.TURRET_CAMERA_RADIUS * Math.sin(turretAngleRad);

        Translation2d turretCameraField = robotPose.transformBy(
            new Transform2d(
                new Translation2d(camX, camY),
                new Rotation2d(turretAngleRad)
            )
        ).getTranslation();

        poseDistance = turretCameraField.getDistance(hubCenter);


        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(turretLimelight);

        double distSum = 0.0;
        double txSum = 0.0;
        double tySum = 0.0;
        double areaSum = 0.0;
        int found = 0;

        for (RawFiducial f : fiducials) {
            if (isHubTag(f.id, hubTagIds)) {
                distSum += f.distToRobot;
                txSum += f.txnc;
                tySum += f.tync;
                areaSum += f.ta;
                found++;
            }
        }

        visibleHubTags = found;

        if (found > 0) {
            limelightDistance = distSum / found;
            targetTX = txSum / found;
            targetTY = tySum / found;
            targetArea = areaSum / found;
            targetDistance = limelightDistance;
            hasLimelightTarget = true;
            usingPoseFallback = false;
        } else {
            limelightDistance = 0.0;
            targetTX = 0.0;
            targetTY = 0.0;
            targetArea = 0.0;
            targetDistance = poseDistance;
            hasLimelightTarget = false;
            usingPoseFallback = true;
        }
    }

    private boolean isHubTag(int id, int[] hubTagIds) {
        for (int hubId : hubTagIds) {
            if (id == hubId) return true;
        }
        return false;
    }
    

    private void processPoseEstimate(PoseEstimate estimate, String cameraName) {
        if (estimate == null || estimate.tagCount == 0) return;

        if (estimate.tagCount >= 2) {
            drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VisionConstants.MULTI_TAG_STD_DEVS);
        } else if (estimate.tagCount == 1 && estimate.avgTagDist < 4.0) {
            drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VisionConstants.SINGLE_TAG_STD_DEVS);
        }

        Logger.recordOutput("Vision/" + cameraName + "RobotPose/", estimate.pose);
        Logger.recordOutput("Vision/" + cameraName + "TagCount/", estimate.tagCount);
        Logger.recordOutput("Vision/" + cameraName + "AvgTagDist/", estimate.avgTagDist);
    }

    public boolean hasTarget() { return hasLimelightTarget || usingPoseFallback; }
    public boolean hasLimelightTarget() { return hasLimelightTarget; }
    public boolean usingPoseFallback() { return usingPoseFallback; }
    public double getTargetTX() { return targetTX; }
    public double getTargetTY() { return targetTY; }
    public double getTargetArea() { return targetArea; }
    public double getTargetDistance() { return targetDistance; }
    public double getLimelightDistance() { return limelightDistance; }
    public double getposeDistance() { return poseDistance; }
    public int getVisibleHubTags() { return visibleHubTags; }
    
    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOff(turretLimelight);
    }
    
    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOn(turretLimelight);
    }
}