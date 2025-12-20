package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.shufflebored.BuiltOutWidgets;

public class SwerveAlignment {
    private Drivetrain m_dDrivetrain;
    public static final String ALIGNMENT_SUGGESTION = "Alignment Suggestion";

    // Front left
    private GenericEntry frontLeftAlignmentDisplayRa;
    private double frontLeftInitialAngle;
    // Front right
    private GenericEntry frontRightAlignmentDisplayRa;
    private double frontRightInitialAngle;
    // Back left
    private GenericEntry backLeftAlignmentDisplayRa;
    private double backLeftInitialAngle;
    // Back right
    private GenericEntry backRightAlignmentDisplayRa;
    private double backRightInitialAngle;

    /**
     * Creates a SwerveAlignment on Shuffleboard
     *
     * @param drivetrain
     */
    public SwerveAlignment(Drivetrain drivetrain) {
        m_dDrivetrain = drivetrain;
    }

    // This section creates the widgets
    public void initSwerveAlignmentWidgets() {
        frontLeftInitialAngle = m_dDrivetrain.getFrontLeftAngle() - DriveConstants.kFrontLeftOffset;
        frontRightInitialAngle = m_dDrivetrain.getFrontRightAngle() - DriveConstants.kFrontRightOffset;
        backLeftInitialAngle = m_dDrivetrain.getBackLeftAngle() - DriveConstants.kBackLeftOffset;
        backRightInitialAngle = m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset;
        frontLeftAlignmentDisplayRa =
                Shuffleboard.getTab(ALIGNMENT_SUGGESTION)
                        .add("FL Test Offset (ra)", m_dDrivetrain.getFrontLeftAngle() - frontLeftInitialAngle)
                        .withPosition(1, 0)
                        .withWidget(BuiltOutWidgets.kRadiableGyro)
                        .getEntry();
        frontRightAlignmentDisplayRa =
                Shuffleboard.getTab(ALIGNMENT_SUGGESTION)
                        .add("FR Test Offset (ra)", m_dDrivetrain.getFrontRightAngle() - frontRightInitialAngle)
                        .withPosition(4, 0)
                        .withWidget(BuiltOutWidgets.kRadiableGyro)
                        .getEntry();
        backLeftAlignmentDisplayRa =
                Shuffleboard.getTab(ALIGNMENT_SUGGESTION)
                        .add("BL Test Offset (ra)", m_dDrivetrain.getBackLeftAngle() - backLeftInitialAngle)
                        .withPosition(1, 3)
                        .withWidget(BuiltOutWidgets.kRadiableGyro)
                        .getEntry();
        backRightAlignmentDisplayRa =
                Shuffleboard.getTab(ALIGNMENT_SUGGESTION)
                        .add("BR Test Offset (ra)", m_dDrivetrain.getBackRightAngle() - backRightInitialAngle)
                        .withPosition(4, 3)
                        .withWidget(BuiltOutWidgets.kRadiableGyro)
                        .getEntry();
    }

    // This section constantly updates the values displayed in the widgets
    public void updateSwerveAlignment() {
        // Front left
        frontLeftAlignmentDisplayRa.setDouble(
                (DriveConstants.kFrontLeftOffset
                                - ((m_dDrivetrain.getFrontLeftAngle() - frontLeftInitialAngle)
                                        - DriveConstants.kFrontLeftOffset))
                        % (2 * Math.PI));
        // Front right
        frontRightAlignmentDisplayRa.setDouble(
                (DriveConstants.kFrontRightOffset
                                - ((m_dDrivetrain.getFrontRightAngle() - frontRightInitialAngle)
                                        - DriveConstants.kFrontRightOffset))
                        % (2 * Math.PI));
        // Back left
        backLeftAlignmentDisplayRa.setDouble(
                (DriveConstants.kBackLeftOffset
                                - ((m_dDrivetrain.getBackLeftAngle() - backLeftInitialAngle)
                                        - DriveConstants.kBackLeftOffset))
                        % (2 * Math.PI));
        // Back right
        backRightAlignmentDisplayRa.setDouble(
                (DriveConstants.kBackRightOffset
                                - ((m_dDrivetrain.getBackRightAngle() - backRightInitialAngle)
                                        - DriveConstants.kBackRightOffset))
                        % (2 * Math.PI));
    }
}
