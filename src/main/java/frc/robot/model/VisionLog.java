package frc.robot.model;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class VisionLog {
    public boolean hasLimelightTarget = false;
    public boolean usingPoseFallback  = false;
    public boolean hasInitializedPose = false;

    public double targetTX           = 0.0;
    public double targetTY           = 0.0;
    public double targetArea         = 0.0;
    public double limelightDistance  = 0.0;
    public double poseDistance       = 0.0;
    public double targetDistance     = 0.0;

    public int visibleHubTags = 0;
}