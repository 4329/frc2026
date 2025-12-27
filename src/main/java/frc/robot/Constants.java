package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utilities.HoorayConfig;

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

        public static final double kVoltCompensation =
                12.0; // Sets a voltage compensation value ideally 12.0V
        public static final double kLoopTime = 20.0;

        public static final int kFrontLeftDriveMotorPort =
                (HoorayConfig.gimmeConfig().getFrontLeftDriveMotorPort()); // CANID of the
        // Translation
        // SparkMAX
        public static final int kFrontRightDriveMotorPort =
                (HoorayConfig.gimmeConfig().getFrontRightDriveMotorPort()); // CANID of the
        // Translation
        // SparkMAX
        public static final int kBackLeftDriveMotorPort =
                (HoorayConfig.gimmeConfig().getBackLeftDriveMotorPort()); // CANID of the
        // Translation
        // SparkMAX
        public static final int kBackRightDriveMotorPort =
                (HoorayConfig.gimmeConfig().getBackRightDriveMotorPort()); // CANID of the
        // Translation
        // SparkMAX
        public static final int kFrontLeftTurningMotorPort =
                (HoorayConfig.gimmeConfig().getFrontLeftTurningMotorPort()); // CANID of
        // the
        // Rotation
        // SparkMAX
        public static final int kFrontRightTurningMotorPort =
                (HoorayConfig.gimmeConfig().getFrontRightTurningMotorPort()); // CANID of
        // the
        // Rotation
        // SparkMAX
        public static final int kBackLeftTurningMotorPort =
                (HoorayConfig.gimmeConfig().getBackLeftTurningMotorPort()); // CANID of the
        // Rotation
        // SparkMAX
        public static final int kBackRightTurningMotorPort =
                (HoorayConfig.gimmeConfig().getBackRightTurningMotorPort()); // CANID of
        // the
        // Rotation
        // SparkMAX

        public static final int kFrontLeftTurningEncoderPort =
                (HoorayConfig.gimmeConfig().getFrontLeftTurningEncoderPort());
        public static final int kFrontRightTurningEncoderPort =
                (HoorayConfig.gimmeConfig().getFrontRightTurningEncoderPort());
        public static final int kBackLeftTurningEncoderPort =
                (HoorayConfig.gimmeConfig().getBackLeftTurningEncoderPort());
        public static final int kBackRightTurningEncoderPort =
                (HoorayConfig.gimmeConfig().getBackRightTurningEncoderPort());

        public static final double kFrontLeftOffset =
                HoorayConfig.gimmeConfig().getFrontLeftOffset(); // Encoder Offset in Radians
        public static final double kFrontRightOffset =
                HoorayConfig.gimmeConfig().getFrontRightOffset(); // Encoder Offset in Radians
        public static final double kBackLeftOffset =
                HoorayConfig.gimmeConfig().getBackLeftOffset(); // Encoder Offset in Radians
        public static final double kBackRightOffset =
                HoorayConfig.gimmeConfig().getBackRightOffset(); // Encoder Offset in Radians

        public static final double[] kFrontLeftTuningVals = {0.0150, 0.2850, 0.25, 0};
        public static final double[] kFrontRightTuningVals = {0.0150, 0.2850, 0.25, 1};
        public static final double[] kBackLeftTuningVals = {0.0150, 0.2850, 0.25, 2};
        public static final double[] kBackRightTuningVals = {0.0150, 0.2850, 0.25, 3};

        // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
        // (Take into consideration when measuring)
        // Center distance in meters between left and right wheels on robot
        public static final double kWheelBaseWidth = 0.629;
        // Center distance in meters between front and back wheels on robot
        public static final double kWheelBaseLength = 0.616;

        // Because the swerve modules position does not change, define a constant
        // SwerveDriveKinematics for use throughout the code
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
                        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
                        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
                        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

        // public static final double kMaxAcceleration = 3.0;
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAngularSpeed = Math.PI;
        public static final double kMaxAngularAccel = Math.PI;

        // public static final double kLowerBound = 0.02;
        public static final double kInnerDeadband = 0.1;
        public static final double kOuterDeadband = 0.98;

        // Minimum allowable rotation command (in radians/s) assuming user input is
        // squared using quadraticTransform, this value is always positive and should be
        // compared against the absolute value of the drive command
        public static final double kMinRotationCommand =
                DriveConstants.kMaxAngularSpeed * Math.pow(DriveConstants.kInnerDeadband, 2);
        // Minimum allowable translation command (in m/s) assuming user input is squared
        // using quadraticTransform, this value is always positive and should be
        // compared against the absolute value of the drive command
        public static final double kMinTranslationCommand =
                DriveConstants.kMaxSpeedMetersPerSecond * Math.pow(DriveConstants.kInnerDeadband, 2);

        public static final double[] kKeepAnglePID = {0.550, 0, 0};
    }

    /** Static method containing all Swerve Module constants */
    public static final class ModuleConstants {
        // Units of %power/s, ie 4.0 means it takes 0.25s to reach 100% power from 0%
        public static final double kTranslationRampRate = 4.0;
        public static final double kTranslationGearRatio = 6.75;

        public static final double kTurningGearRatio = 21.428571428571428571428571428571;

        public static final double kTurningPositionFactor = 1.0 / kTurningGearRatio * 2.0 * Math.PI;
        public static final double kTurningVelocityFactor =
                1.0 / kTurningGearRatio / 60 * 2.0 * Math.PI;

        private static final double kWheelDiameter = 0.09845; // Wheel Diameter in meters

        public static final double kVelocityFactor =
                (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI;

        public static final double kPositionFactor =
                1.0 / kTranslationGearRatio * (kWheelDiameter * Math.PI);

        // NOTE: You shoulds ALWAYS define a reasonable current limit when using
        // brushless motors due to the extremely high stall current available
        public static final int kDriveCurrentLimit = 60;
        public static final int kTurnCurrentLimit = 60;

        public static final double[] kTurnPID = {
            0.65, 0, 0
        }; // should show some minor oscillation when no weight is
        // loaded on the modules
    }

    /** Static method containing all User O/I constants */
    public static final class OIConstants {

        public static final int kDriverControllerPort =
                0; // When making use of multiple controllers for drivers each
        // controller will be on a different port
        public static final int kOperatorControllerPort =
                1; // When making use of multiple controllers for drivers each
        // controller will be on a different port
        public static final int kManualControllerPort =
                2; //  When making blah blah blah make the ports different!
        public static final int kFunctionalControllerPort = 5;
    }

    /*
     * Static method containing all Autonomous constants
     */
    public static final class AutoConstants {
        public static PIDConstants translationPID = new PIDConstants(2, 0, 0.00005);
        public static PIDConstants rotationPID = new PIDConstants(1.25, 0, 0);
        public static PathFollowingController ppHolonomicDriveController =
                new PPHolonomicDriveController(
                        Constants.AutoConstants.translationPID, Constants.AutoConstants.rotationPID);

        public static RobotConfig config;
    }

    public static final class SparkIDs {
        public static final int differential1 = 13;
        public static final int differential2 = 14;

        public static final int algeePivot = 11;
        public static final int algeeWheel = 12;

        public static final int elevator1 = 9;
        public static final int elevator2 = 10;

        public static final int intakePivot = 15;
        public static final int intakeWheel = 16;

        public static final int pigeon = 29;

        public static final int climber = 17;
    }
}
