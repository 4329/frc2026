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
        
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    }
    
    @Override
    public void periodic() {
        var pigeon = drivetrain.getPigeon2();
        double yaw = pigeon.getYaw().getValueAsDouble();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            yaw,
            yawRate,
            0, 0, 0, 0  
        );
        
        boolean tv = LimelightHelpers.getTV(limelightName);
        double tid = LimelightHelpers.getFiducialID(limelightName);
        

        if (tv && (int)tid == VisionConstants.TARGET_TAG_ID) {
            hasTarget = true;
            targetTX = LimelightHelpers.getTX(limelightName);
            targetTY = LimelightHelpers.getTY(limelightName);
            targetArea = LimelightHelpers.getTA(limelightName);
            
            RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightName);
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
        

        PoseEstimate poseEstimate;
        var alliance = DriverStation.getAlliance();
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
        } else {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        }
        
        if (poseEstimate != null && poseEstimate.tagCount > 0) {
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
            
            Logger.recordOutput("Vision/RobotPose", poseEstimate.pose);
            Logger.recordOutput("Vision/TagCount", poseEstimate.tagCount);
            Logger.recordOutput("Vision/AverageDist", poseEstimate.avgTagDist);
        }
        
        Logger.recordOutput("Vision/TargetFound", hasTarget);
        Logger.recordOutput("Vision/TargetTX", targetTX);
        Logger.recordOutput("Vision/TargetTY", targetTY);
        Logger.recordOutput("Vision/TargetArea", targetArea);
        Logger.recordOutput("Vision/TargetDistance", targetDistance);
    }
    

    public boolean hasTarget() {
        return hasTarget;
    }
    

    public double getTargetTX() {
        return targetTX;
    }
    

    public double getTargetTY() {
        return targetTY;
    }


    public double getTargetArea() {
        return targetArea;
    }
    

    public double getTargetDistance() {
        return targetDistance;
    }
    
    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }
    
    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }
}