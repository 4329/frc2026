// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Drive5mAuto;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.CommandGroups.TurretSubsystemCommandGroupMax;
import frc.robot.commands.CommandGroups.TurretSubsystemCommandGroupMin;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private boolean isFieldCentric = true;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final HoodSubsystem hood = new HoodSubsystem();

    private final RotateSubsystem turret = new RotateSubsystem();

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private final NEO550ThroughTalonFXSSubsytem spinner = new NEO550ThroughTalonFXSSubsytem();

    private final Field2d field = new Field2d();

    Map<Command, PathPlannerAuto> autoName = new HashMap<>();

    public RobotContainer() {

        drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180));

        SmartDashboard.putData("Field", field);

        configureBindings();
    }

    public void robotPeriodic() {
        // Log driver controller inputs
        Logger.recordOutput("Controller/Driver/LeftX", joystick.getLeftX());
        Logger.recordOutput("Controller/Driver/AButton", joystick.a().getAsBoolean());

        field.setRobotPose(drivetrain.getState().Pose);

        Logger.recordOutput("Drivetrain/Pose", drivetrain.getState().Pose);
        Logger.recordOutput("Drivetrain/Speeds", new double[] {
            drivetrain.getState().Speeds.vxMetersPerSecond,
            drivetrain.getState().Speeds.vyMetersPerSecond,
            drivetrain.getState().Speeds.omegaRadiansPerSecond
        });
    }



    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, joystick::getLeftX, joystick::getLeftY, joystick::getRightX, () -> isFieldCentric, MaxSpeed, MaxAngularRate));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.b().whileTrue(new TurretSubsystemCommandGroupMax(turret, hood, shooter));

        joystick.a().whileTrue(new TurretSubsystemCommandGroupMin(turret, hood, shooter));

        joystick.leftBumper().whileTrue(new FollowAprilTagCommand(vision, drivetrain));
        
        // Temporary test - print when button pressed
        joystick.rightBumper().onTrue(Commands.runOnce(() -> {
            System.out.println("=== VISION TEST ===");
            System.out.println("Has target: " + vision.hasTarget());
            System.out.println("TX: " + vision.getTargetTX());
            System.out.println("TY: " + vision.getTargetTY());
            System.out.println("Distance: " + vision.getTargetDistance());
    
            // Check raw Limelight data
            System.out.println("LL TV: " + LimelightHelpers.getTV("limelight-swerve"));
            System.out.println("LL TX: " + LimelightHelpers.getTX("limelight-swerve"));
        }));

        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.povDown().onTrue(Commands.runOnce(() -> isFieldCentric = !isFieldCentric));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        drivetrain.registerTelemetry(logger::telemeterize);                         
    }

    public Command getAutonomousCommand() {
        return Drive5mAuto.create(drivetrain);
    }

    public String getAutoName(Command command) {
    return autoName.containsKey(command) ? autoName.get(command).getName() : "Nothing?????/?///?";
    }
}
