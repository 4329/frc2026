package frc.robot;

import javax.management.RuntimeErrorException;

import org.opencv.core.Mat;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
// import frc.robot.utilities.HoorayConfig;

/** Static method containing all constant values for the robot in one location */
public final class Constants {

    public static Mode robotMode;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final double voltageCompensation = 12.0;

    /** Static method containing all Drivetrain constants */
    public static final class DriveConstants {
        public static final double kVoltCompensation = 12.0;
        public static final double kLoopTime = 20.0;

        public static double kMaxSpeedMultiplier = 1.0;
        public static double kMaxAngularRotationsPerSecond = 0.75;

        public static final double kFieldCentricSeedAngleDegrees = 180;

        public static final double kWheelBaseWidth = 0.629;
        public static final double kWheelBaseLength = 0.616;

        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
                        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
                        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
                        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAngularSpeed = Math.PI;
        public static final double kMaxAngularAccel = Math.PI;

        public static final double kInnerDeadband = 0.1;
        public static final double kOuterDeadband = 0.98;

        public static final double[] kKeepAnglePID = {0.550, 0, 0};
    }

    /** Static method containing all Swerve Module constants */
    public static final class ModuleConstants {
        public static final double kTranslationRampRate = 4.0;
        public static final double kTranslationGearRatio = 6.75;
        public static final double kTurningGearRatio = 21.428571428571428571428571428571;

        public static final double kTurningPositionFactor = 1.0 / kTurningGearRatio * 2.0 * Math.PI;
        public static final double kTurningVelocityFactor = 1.0 / kTurningGearRatio / 60 * 2.0 * Math.PI;

        private static final double kWheelDiameter = 0.09845;

        public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI;
        public static final double kPositionFactor = 1.0 / kTranslationGearRatio * (kWheelDiameter * Math.PI);

        public static final int kDriveCurrentLimit = 60;
        public static final int kTurnCurrentLimit = 60;

        public static final double[] kTurnPID = { 0.65, 0, 0 };
    }

    /** * NEW: Advanced Intake Tuning Constants 
     * These are used by the IntakeSubsystem for smart movement.
     */
    public static final class IntakeConstants {
        // Position Setpoints (Rotations)
        public static final double kRetractedPos = 0.1; 
        public static final double kExtendedPos = 24.5;

        // MAXMotion Profile Constraints (2026 Season Tuning)
        public static final double kMaxVel = 55.0; // rps
        public static final double kMaxAcc = 110.0; // rps^2
        
        // Gains
        public static final double kP = 0.15;
        public static final double kG = 0.06; // Volts needed to fight gravity

        // Piece Detection & Speed
        public static final double kIntakeSpeed = 0.85;
        public static final double kPieceCurrentThreshold = 38.0; // Amps on Kraken
        public static final double kHomingCurrent = 15.0; // Amps for stall homing
        public static double kHomingSpeed;
        public static double kRollerCurrentLimit;
        public static int kPivotCurrentLimit;
        public static double kI;
        public static double kD;
    }

    /** Static method containing all User O/I constants */
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kManualControllerPort = 2;
        public static final int kFunctionalControllerPort = 5;
    }

    /** Static method containing all Autonomous constants */
    public static final class AutoConstants {
        public static PIDConstants translationPID = new PIDConstants(2, 0, 0.00005);
        public static PIDConstants rotationPID = new PIDConstants(1.25, 0, 0);
        public static PathFollowingController ppHolonomicDriveController =
                new PPHolonomicDriveController(
                        Constants.AutoConstants.translationPID, Constants.AutoConstants.rotationPID);

        public static RobotConfig config;
    }

    /** Static method containing all CAN IDs */
    public static final class SparkIDs {
        public static final int differential1 = 13;
        public static final int differential2 = 14;

        public static final int algeePivot = 11;
        public static final int algeeWheel = 12;

        public static final int elevator1 = 9;
        public static final int elevator2 = 10;

        // These IDs are shared by both your old Spinner and new Intake subsystems
        public static final int intakePivot = 15;
        public static final int intakeWheel = 16;

        public static final int pigeon = 29;
        public static final int climber = 17;
    }


    public static final class VisionConstants {

        public static final String LIMELIGHT_SWERVE_NAME = "limelight-swerve";
        public static final String LIMELIGHT_TURRET_NAME = "limelight-turret";


        public static final Transform3d ROBOT_TO_SWERVE_CAMERA = new Transform3d(new Translation3d(-0.3478923, 0.2105523, 0.204579), new Rotation3d(Math.toRadians(180), Math.toRadians(20), Math.toRadians(180)));
        public static final Transform3d ROBOT_TO_TURRET_CAMERA = new Transform3d(new Translation3d(-0.295275, -0.1143, 0.5461), new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(180)));   

        public static final double TURRET_CAMERA_RADIUS = 0.147066;

        public static final double TURRET_AXIS_X = -0.157988;
        public static final double TURRET_AXIS_Y = -0.108458;

        public static final Vector<N3> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.0, 1.0, 2.0);
        public static final Vector<N3> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0); 


        public static final int[] RED_HUB_TAG_IDS = { 2, 3, 4, 5, 8, 9, 10, 11 };
        public static final int[] BLUE_HUB_TAG_IDS = { 18, 19, 20, 21, 24, 25, 26, 27 };

        public static final AprilTagFieldLayout FIELD_LAYOUT;

        public static final Translation2d BLUE_HUB_CENTER;
        public static final Translation2d RED_HUB_CENTER;

        static {
            AprilTagFieldLayout layout;
            try{
                layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            } catch (Exception e) {
                throw new RuntimeException("Failed to load 2026 Apriltag Field Layout", e);
            }
            FIELD_LAYOUT = layout;

            BLUE_HUB_CENTER = computeHubCenter(layout, BLUE_HUB_TAG_IDS);
            RED_HUB_CENTER = computeHubCenter(layout, RED_HUB_TAG_IDS);
        }

        private static Translation2d computeHubCenter (AprilTagFieldLayout layout, int[] tagIds) {
            double xSum = 0;
            double ySum = 0;
            int count = 0;
            for (int id : tagIds) {
                var pose = layout.getTagPose(id);
                if (pose.isPresent()) {
                    xSum += pose.get().getX();
                    ySum += pose.get().getY();
                    count++;
                }
            }
            if (count == 0) throw new RuntimeException("No Hub tags found in field layout!");
            return new Translation2d(xSum / count, ySum / count);
        }


        public static final double FOLLOW_DISTANCE_METERS = 1.5;
        public static final double FOLLOW_SPEED = 0.5;
        public static final double FOLLOW_KP_TRANSLATION = 0.2;
        public static final double FOLLOW_KP_ROTATION = 0.02;
        public static final double DISTANCE_TOLERANCE = 0.1;
        public static final double ANGLE_TOLERANCE = 2.0;
    }
}
