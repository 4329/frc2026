package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    
    private boolean hasTarget = false;
    private double targetTX = 0.0;
    private double targetTY = 0.0;
    private double targetArea = 0.0;
    private double targetDistance = 0.0;
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.limelightName = VisionConstants.LIMELIGHT_NAME;
        
        // Set camera pose
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            VisionConstants.ROBOT_TO_CAMERA.getTranslation().getX(),  // Forward
            -VisionConstants.ROBOT_TO_CAMERA.getTranslation().getY(), // Left (negate Y)
            VisionConstants.ROBOT_TO_CAMERA.getTranslation().getZ(),  // Up
            Math.toDegrees(VisionConstants.ROBOT_TO_CAMERA.getRotation().getX()), // Roll
            Math.toDegrees(VisionConstants.ROBOT_TO_CAMERA.getRotation().getY()), // Pitch
            Math.toDegrees(VisionConstants.ROBOT_TO_CAMERA.getRotation().getZ())  // Yaw
        );
        
        // Set LEDs to pipeline control
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    }
    
    @Override
    public void periodic() {
        // Update robot orientation for MegaTag2
        var pigeon = drivetrain.getPigeon2();
        double yaw = pigeon.getYaw().getValueAsDouble();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            yaw,
            yawRate,
            0, 0, 0, 0  // pitch, pitchRate, roll, rollRate
        );
        
        // Get raw NetworkTables data directly (bypasses JSON)
        boolean tv = LimelightHelpers.getTV(limelightName);
        double tid = LimelightHelpers.getFiducialID(limelightName);
                
        // Check if we see our target tag
        if (tv && (int)tid == VisionConstants.TARGET_TAG_ID) {
            hasTarget = true;
            targetTX = LimelightHelpers.getTX(limelightName);
            targetTY = LimelightHelpers.getTY(limelightName);
            targetArea = LimelightHelpers.getTA(limelightName);
            
            // Get distance from raw fiducials
            RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightName);
            targetDistance = 0.0;
            for (RawFiducial fiducial : rawFiducials) {
                if (fiducial.id == VisionConstants.TARGET_TAG_ID) {
                    targetDistance = fiducial.distToRobot;
                    break;
                }
            }
            
            // Fallback distance calculation if raw fiducials not available
            if (targetDistance == 0.0 && targetArea > 0.1) {
                targetDistance = Math.sqrt(5.0 / targetArea);
            }
        
        // Get MegaTag2 pose estimate
        PoseEstimate poseEstimate = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        
        if (poseEstimate != null && poseEstimate.tagCount > 0) {
            // Add vision measurement to drivetrain pose estimator
            if (poseEstimate.tagCount >= 2) {
                // Multiple tags = higher confidence
                drivetrain.addVisionMeasurement(
                    poseEstimate.pose,
                    poseEstimate.timestampSeconds,
                    VisionConstants.MULTI_TAG_STD_DEVS
                );
            } else if (poseEstimate.tagCount == 1 && poseEstimate.avgTagDist < 4.0) {
                // Single tag within 4m = lower confidence
                drivetrain.addVisionMeasurement(
                    poseEstimate.pose,
                    poseEstimate.timestampSeconds,
                    VisionConstants.SINGLE_TAG_STD_DEVS
                );
            }
            
            // Log vision data
            Logger.recordOutput("Vision/RobotPose", poseEstimate.pose);
            Logger.recordOutput("Vision/TagCount", poseEstimate.tagCount);
            Logger.recordOutput("Vision/AverageDist", poseEstimate.avgTagDist);
        }
        
        // Log target data
        Logger.recordOutput("Vision/TargetFound", hasTarget);
        Logger.recordOutput("Vision/TargetTX", targetTX);
        Logger.recordOutput("Vision/TargetTY", targetTY);
        Logger.recordOutput("Vision/TargetArea", targetArea);
        Logger.recordOutput("Vision/TargetDistance", targetDistance);
    }
}
    
    /**
     * Check if we can see the target AprilTag
     */
    public boolean hasTarget() {
        return hasTarget;
    }
    
    /**
     * Get horizontal offset to target in degrees
     */
    public double getTargetTX() {
        return targetTX;
    }
    
    /**
     * Get vertical offset to target in degrees
     */
    public double getTargetTY() {
        return targetTY;
    }
    
    /**
     * Get target area (0-100)
     */
    public double getTargetArea() {
        return targetArea;
    }
    
    /**
     * Get distance to target tag in meters
     */
    public double getTargetDistance() {
        return targetDistance;
    }
    
    /**
     * Set LED mode
     */
    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }
    
    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }
}