package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean isTargetFresh = false;
    private double targetTX = 0.0;
    private double targetTY = 0.0;
    private double targetArea = 0.0;
    private double targetDistance = 0.0;
    private int visibleHubTags = 0;
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
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

        LimelightHelpers.setCameraPose_RobotSpace(
            turretLimelight,
            VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getX(),  // Forward
            -VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getY(), // Left (negate Y)
            VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getZ(),  // Up
            Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getX()), // Roll
            Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getY()), // Pitch
            Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getZ())  // Yaw
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

        updateTurretTrageting();
        
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
        
        Logger.recordOutput("Vision/TargetFound", hasTarget);
        Logger.recordOutput("Vision/TargetFresh", isTargetFresh);
        Logger.recordOutput("Vision/TargetTX", targetTX);
        Logger.recordOutput("Vision/TargetTY", targetTY);
        Logger.recordOutput("Vision/TargetArea", targetArea);
        Logger.recordOutput("Vision/TargetDistance", targetDistance);
        Logger.recordOutput("Vision/VisibleHubTags", visibleHubTags);

        SmartDashboard.putBoolean("Vision/Hub Target Locked", hasTarget);
        SmartDashboard.putBoolean("Vision/Distance Fresh", isTargetFresh);
        SmartDashboard.putNumber("Vision/Hub Distance (m)", targetDistance);
        SmartDashboard.putNumber("Vision/Hub Tags Visible", visibleHubTags);
        SmartDashboard.putNumber("Vision/Target TX", targetTX);
        SmartDashboard.putNumber("Vision/Target TY", targetTY);
    }

    private void updateTurretTrageting() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(turretLimelight);

        double distanceSum = 0.0;
        double txSum = 0.0;
        double tySum = 0.0;
        double areaSum = 0.0;
        int hubTagsFound = 0;

        for (RawFiducial f : fiducials) {
            if (isHubTag(f.id)) {
                distanceSum += f.distToRobot;
                txSum += f.txnc;
                tySum += f.tync;
                areaSum += f.ta;
                hubTagsFound++;
            }
        }

        visibleHubTags = hubTagsFound;

        if (hubTagsFound > 0) {
            hasTarget = true;
            isTargetFresh = true;
            targetDistance = distanceSum / hubTagsFound;
            targetTX = txSum / hubTagsFound;
            targetTY = tySum / hubTagsFound;
            targetArea = areaSum / hubTagsFound;
        } else {
            hasTarget = false;
            isTargetFresh = false;
            targetTX = 0.0;
            targetTY = 0.0;
            targetArea = 0.0;
        }
    }

    private boolean isHubTag(int id) {
        for (int hubId : VisionConstants.HUB_TAG_IDS) {
            if (id == hubId) return true;
        }
        return false;
    }
    

    private void processPoseEstimate(PoseEstimate poseEstimate, String cameraName) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) return;

        if (poseEstimate.tagCount >= 2) {
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, VisionConstants.MULTI_TAG_STD_DEVS);
        } else if (poseEstimate.tagCount == 1 && poseEstimate.avgTagDist < 4.0) {
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, VisionConstants.SINGLE_TAG_STD_DEVS);
        }

        Logger.recordOutput("Vision/" + cameraName + "RobotPose/", poseEstimate.pose);
        Logger.recordOutput("Vision/" + cameraName + "RobotPose/", poseEstimate.tagCount);
        Logger.recordOutput("Vision/" + cameraName + "RobotPose/", poseEstimate.avgTagDist);
    }

    public boolean hasTarget() { return hasTarget; }
    public boolean isTargetFresh() { return isTargetFresh; }
    public double getTargetTX() { return targetTX; }
    public double getTargetTY() { return targetTY; }
    public double getTargetArea() { return targetArea; }
    public double getTargetDistance() { return targetDistance; }
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