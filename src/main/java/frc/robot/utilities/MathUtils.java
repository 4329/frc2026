package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.*;

/**
 * This file is created to hold any particularly useful math utilies which may be called statically
 */
public class MathUtils {
    // Converts a radian angle into the unit circle range of 0 to 2PI
    /**
     * This function takes in an angle in radians and does the proper math to convert the output to an
     * angle from 0 to 2PI also known as a "unit circle angle"
     *
     * @param angle is the input angle in radians
     * @return a unit circle angle equal to the input angle
     */
    public static double toUnitCircAngle(double angle) {
        double rotations = angle / (2 * Math.PI);
        return (angle - Math.round(rotations - 0.500) * Math.PI * 2.0);
    }

    /**
     * Takes an input value and squares it, but retains the sign. IE negative numbers will remain
     * negative.
     *
     * @param input is the number to perform the transform on
     * @return the transformed input value
     */
    public static double singedSquare(double input) {
        return Math.signum(input) * Math.pow(input, 2);
    }

    public static double singedPow(double input, double power) {
        return Math.signum(input) * Math.abs(Math.pow(input, power));
    }

    /**
     * Applies a simple deadband to input values between -1.0 and 1.0. Makes use of the deadband
     * constants stored in the constants class. Values below the innerDeadband will be set to zero.
     * Values above the outerDeadband will be set to 1.0 or -1.0 depending on the sign of the input
     *
     * @param input is the input to perform the deadband on
     * @return the resulting input after applying the deadband
     */
    public static double applyDeadband(double input) {
        if (Math.abs(input) > DriveConstants.kOuterDeadband) return Math.signum(input);
        else if (Math.abs(input) < DriveConstants.kInnerDeadband) return 0;
        else return input;
        // double tmput;
        // if (Math.abs(input) < DriveConstants.kInnerDeadband - DriveConstants.kLowerBound) tmput = 0;
        // else if (Math.abs(input) > DriveConstants.kOuterDeadband) tmput = 1;
        // else
        //    tmput =
        //            Math.abs(
        //                    (input + DriveConstants.kInnerDeadband)
        //                            * (DriveConstants.kOuterDeadband -
        // DriveConstants.kInnerDeadband));
        //
        // return Math.signum(input) * tmput;
    }

    /**
     * Turns inches into meters
     *
     * @param inches
     * @return meters
     */
    public static double inchesToMeters(double inches) {

        return inches * 0.0254;
    }

    /**
     * Turns meters into inches
     *
     * @param meters
     * @return inches
     */
    public static double metersToInches(double meters) {

        return meters * 39.37;
    }

    public static double clamp(double min, double max, double value) {
        return Math.min(max, Math.max(min, value));
    }

    public static Pose2d transform2dToPose2d(Transform2d transform) {
        return new Pose2d(transform.getX(), transform.getY(), transform.getRotation());
    }

    public static Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static Transform2d transform3dToTransform2d(Transform3d transform) {
        return new Transform2d(
                transform.getX(), transform.getZ(), transform.getRotation().toRotation2d());
    }

    public static Pose3d pose2DtoPose3D(Pose2d pose) {
        return new Pose3d(
                pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
    }

    public static Pose3d addPoses3D(Pose3d one, Pose3d two) {
        Translation3d translation3d = one.getTranslation().plus(two.getTranslation());
        Rotation3d rotation3d = one.getRotation().plus(two.getRotation());
        return new Pose3d(translation3d, rotation3d);
    }

    public static double getActualDistanceFromPose(Pose3d pose) {

        return Math.sqrt(Math.pow(pose.getZ(), 2) + Math.pow(pose.getX(), 2));
    }
}
