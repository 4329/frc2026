package frc.robot.model;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

@AutoLog
public class DrivetrainLog {
    public Pose2d pose = new Pose2d();
    public LinearVelocity velocityX = MetersPerSecond.of(0);
    public LinearVelocity velocityY = MetersPerSecond.of(0);
    public AngularVelocity angularVelocity = RadiansPerSecond.of(0);
    public Rotation2d gyroAngle = new Rotation2d();
}
