package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;

public class VisionSubsystem extends SubsystemBase {

    // Field dimensions for blue-origin coordinate conversion
    private static final double FIELD_LENGTH = 16.5412; // meters
    private static final double FIELD_WIDTH  = 8.0137;  // meters

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

    private boolean hasInitializedPose = false;


    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, RotateSubsystem turret) {
        this.drivetrain      = drivetrain;
        this.turret          = turret;
        this.swerveLimelight = VisionConstants.LIMELIGHT_SWERVE_NAME;
        this.turretLimelight = VisionConstants.LIMELIGHT_TURRET_NAME;

        // Swerve limelight is stationary — set once at startup
        LimelightHelpers.setCameraPose_RobotSpace(
            swerveLimelight,
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getX(),
            -VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getY(),
            VisionConstants.ROBOT_TO_SWERVE_CAMERA.getTranslation().getZ(),
            180.0,
            20.0,
            180.0
        );

        // Turret limelight pose is updated every loop in periodic()
        LimelightHelpers.setPipelineIndex(swerveLimelight, 0);
        LimelightHelpers.setPipelineIndex(turretLimelight, 0);

        LimelightHelpers.setLEDMode_PipelineControl(swerveLimelight);
        LimelightHelpers.setLEDMode_PipelineControl(turretLimelight);
    }

    @Override
    public void periodic() {
        var pigeon = drivetrain.getPigeon2();

        // double pigeonYaw = pigeon.getYaw().getValueAsDouble();
        double headingDeg = drivetrain.getState().Pose.getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(swerveLimelight, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(turretLimelight, headingDeg, 0, 0, 0, 0, 0);

        // Update turret limelight pose every loop based on current turret angle
        updateTurretCameraPose();

        // Always use wpiBlue — MT2 always outputs blue-origin coordinates
        processPoseEstimate(
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(swerveLimelight),
            "Swerve"
        );
        processPoseEstimate(
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(turretLimelight),
            "Turret"
        );

        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false);

        // Hub tag IDs based on alliance — these are the tags we actually look for
        int[] hubTagIds = isRed
            ? VisionConstants.RED_HUB_TAG_IDS
            : VisionConstants.BLUE_HUB_TAG_IDS;

        // Hub center in the same coordinate system as the drivetrain pose estimator.
        // On red, drivetrain is in red-origin so use RED_HUB_CENTER directly.
        // On blue, drivetrain is in blue-origin so use BLUE_HUB_CENTER directly.
        Translation2d hubCenter = isRed
            ? VisionConstants.RED_HUB_CENTER
            : VisionConstants.BLUE_HUB_CENTER;

        updateTurretTargeting(hubCenter, hubTagIds);

        // --- AdvantageKit logging ---
        Logger.recordOutput("Vision/HasLimelightTarget", hasLimelightTarget);
        Logger.recordOutput("Vision/UsingPoseFallback",  usingPoseFallback);
        Logger.recordOutput("Vision/LimelightDistance",  limelightDistance);
        Logger.recordOutput("Vision/PoseDistance",       poseDistance);
        Logger.recordOutput("Vision/TargetDistance",     targetDistance);
        Logger.recordOutput("Vision/VisibleHubTags",     visibleHubTags);
        Logger.recordOutput("Vision/TargetTX",           targetTX);
        Logger.recordOutput("Vision/TargetTY",           targetTY);
        Logger.recordOutput("Vision/HubCenter",          hubCenter);

        // --- Elastic / SmartDashboard ---
        SmartDashboard.putBoolean("Vision/Hub Target Locked",     hasLimelightTarget);
        SmartDashboard.putBoolean("Vision/Using Pose Fallback",   usingPoseFallback);
        SmartDashboard.putNumber("Vision/Hub Distance (m)",       targetDistance);
        SmartDashboard.putNumber("Vision/Limelight Distance (m)", limelightDistance);
        SmartDashboard.putNumber("Vision/Pose Distance (m)",      poseDistance);
        SmartDashboard.putNumber("Vision/Hub Tags Visible",       visibleHubTags);
        SmartDashboard.putNumber("Vision/Target TX",              targetTX);
        SmartDashboard.putNumber("Vision/Target TY",              targetTY);
        SmartDashboard.putString("Vision/Alliance",               isRed ? "RED" : "BLUE");

        // --- Debug ---
        SmartDashboard.putNumber("Debug/PigeonRawYaw",   headingDeg);
        SmartDashboard.putNumber("Debug/RobotPoseX",     drivetrain.getState().Pose.getTranslation().getX());
        SmartDashboard.putNumber("Debug/RobotPoseY",     drivetrain.getState().Pose.getTranslation().getY());
        SmartDashboard.putNumber("Debug/BlueHubCenterX", VisionConstants.BLUE_HUB_CENTER.getX());
        SmartDashboard.putNumber("Debug/BlueHubCenterY", VisionConstants.BLUE_HUB_CENTER.getY());
        SmartDashboard.putNumber("Debug/RedHubCenterX",  VisionConstants.RED_HUB_CENTER.getX());
        SmartDashboard.putNumber("Debug/RedHubCenterY",  VisionConstants.RED_HUB_CENTER.getY());
    }

    private void updateTurretCameraPose() {
        double turretAngleRad = turret.getPosition() * 2.0 * Math.PI;

        double camOffsetX = VisionConstants.TURRET_AXIS_X
            + VisionConstants.TURRET_CAMERA_RADIUS * Math.cos(turretAngleRad);
        double camOffsetY = VisionConstants.TURRET_AXIS_Y
            + VisionConstants.TURRET_CAMERA_RADIUS * Math.sin(turretAngleRad);

        double camZ     = VisionConstants.ROBOT_TO_TURRET_CAMERA.getTranslation().getZ();
        double camPitch = Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getY());
        double camRoll  = Math.toDegrees(VisionConstants.ROBOT_TO_TURRET_CAMERA.getRotation().getX());

        // Yaw rotates with turret — add 180° since camera faces backward at position 0
        double camYaw = Math.toDegrees(turretAngleRad) + 180.0;

        LimelightHelpers.setCameraPose_RobotSpace(
            turretLimelight,
            camOffsetX,
            -camOffsetY,
            camZ,
            camRoll,
            camPitch,
            camYaw
        );
    }

    private void updateTurretTargeting(Translation2d hubCenter, int[] hubTagIds) {
        // Compute turret camera's current field position dynamically
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

        // Pose-based distance from turret camera to hub center (both in blue-origin)
        poseDistance = turretCameraField.getDistance(hubCenter);

        // Try limelight-based distance from visible hub tags
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
        if (estimate == null || estimate.tagCount == 0) return;

        if (!hasInitializedPose && estimate.tagCount >= 2) {
            drivetrain.resetPose(estimate.pose);
            hasInitializedPose = true;
            Logger.recordOutput("Vision/PoseInitialized", true);
            return;
        }

        // boolean isRed = DriverStation.getAlliance()
        //     .map(a -> a == Alliance.Red)
        //     .orElse(false);

        // MT2 always outputs blue-origin poses.
        // setOperatorPerspectiveForward in CommandSwerveDrivetrain rotates the pose
        // estimator into red-origin on red alliance, so we need to flip the pose
        // to match whichever coordinate system the drivetrain is currently using.

        // Pose2d pose = isRed ? flipToRedOrigin(estimate.pose) : estimate.pose;
        // Pose2d currentPose = drivetrain.getState().Pose;
        // double poseError = currentPose.getTranslation()
        //     .getDistance(estimate.pose.getTranslation());
        // if (poseError > 1.0) return;


        Pose2d pose = estimate.pose;

        if (estimate.tagCount >= 2) {
            drivetrain.addVisionMeasurement(
                pose, estimate.timestampSeconds, VisionConstants.MULTI_TAG_STD_DEVS);
        } else if (estimate.tagCount == 1 && estimate.avgTagDist < 4.0) {
            drivetrain.addVisionMeasurement(
                pose, estimate.timestampSeconds, VisionConstants.SINGLE_TAG_STD_DEVS);
        }

        Logger.recordOutput("Vision/" + cameraName + "/RobotPose",  estimate.pose);
        Logger.recordOutput("Vision/" + cameraName + "/TagCount",   estimate.tagCount);
        Logger.recordOutput("Vision/" + cameraName + "/AvgTagDist", estimate.avgTagDist);
    }

    public void resetPoseInitialization() {
        hasInitializedPose = false;
    }

    // Converts a blue-origin pose to red-origin by mirroring across the field center
    private Pose2d flipToRedOrigin(Pose2d pose) {
        return new Pose2d(
            FIELD_LENGTH - pose.getX(),
            FIELD_WIDTH  - pose.getY(),
            pose.getRotation().plus(Rotation2d.fromDegrees(180))
        );
    }

    // --- Public getters ---
    public boolean hasTarget()            { return hasLimelightTarget || usingPoseFallback; }
    public boolean hasLimelightTarget()   { return hasLimelightTarget; }
    public boolean isUsingPoseFallback()  { return usingPoseFallback; }
    public double  getTargetTX()          { return targetTX; }
    public double  getTargetTY()          { return targetTY; }
    public double  getTargetArea()        { return targetArea; }
    public double  getTargetDistance()    { return targetDistance; }
    public double  getLimelightDistance() { return limelightDistance; }
    public double  getPoseDistance()      { return poseDistance; }
    public int     getVisibleHubTags()    { return visibleHubTags; }

    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOff(turretLimelight);
    }

    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(swerveLimelight);
        LimelightHelpers.setLEDMode_ForceOn(turretLimelight);
    }
}