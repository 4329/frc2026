// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.commands.PositionSpinNEO550Command;
import frc.robot.commands.SetHoodPositionCommand;
import frc.robot.commands.ShooterVolSpinCommand;
import frc.robot.commands.TurretPositionWithSpeedCommand;
import frc.robot.commands.VoltageSpinNEO550Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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

    private final NEO550ThroughTalonFXSSubsytem spinner = new NEO550ThroughTalonFXSSubsytem();

    private final HoodSubsystem hood = new HoodSubsystem();

    private final TurretSubsystem turret = new TurretSubsystem();

    private final ShooterSubsystem shooter = new ShooterSubsystem();

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

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        //joystick.a().onTrue(new PositionSpinNEO550Command(spinner, 0.0));
        //joystick.b().onTrue(new PositionSpinNEO550Command(spinner, 20.0));
        //joystick.x().whileTrue(new VoltageSpinNEO550Command(spinner, 6.0));
        //joystick.y().whileTrue(new VoltageSpinNEO550Command(spinner, -6.0));

        // joystick.a().onTrue(new PosHoodSpinCommand(motorYes, 0.25));
        // joystick.b().onTrue(new PosKrakenMotorSpinCommand(motorYes, 0.0));
        // joystick.x().whileTrue(new VolHoodSpinCommand(motorYes, 0.5));
        // joystick.y().whileTrue(new VolHoodSpinCommand(motorYes, -0.5));

        // joystick.a().onTrue(new SetHoodPositionCommand(hood, 0.05)); // Minimum position
        // joystick.b().onTrue(new SetHoodPositionCommand(hood, 0.25)); // Mid position
        // joystick.povRight().onTrue(new SetHoodPositionCommand(hood, 0.44));
        // joystick.povLeft().onTrue(new SetHoodPositionCommand(hood, 1.0));


        // joystick.a().onTrue(new TurretPositionWithSpeedCommand(turret, 1.2));
        // joystick.b().onTrue(new TurretPositionWithSpeedCommand(turret, -1.2));
        // joystick.x().onTrue(new TurretPositionWithSpeedCommand(turret, 0));
        // joystick.y().onTrue(new SetTurretZeroCommand(turret));
        // joystick.x().whileTrue(new ShooterVolSpinCommand(shooter, 0.5));
        // joystick.y().whileTrue(new ShooterVolSpinCommand(shooter, 0.5));

        // joystick.a().onTrue(new TurretPositionWithSpeedCommand(turret, 1));
        // joystick.a().onTrue(new SetHoodPositionCommand(hood, 0.2));
        joystick.a().whileTrue(new ShooterVolSpinCommand(shooter, 0.5));

        joystick.b().onTrue(new TurretPositionWithSpeedCommand(turret, -0.7));
        joystick.b().onTrue(new SetHoodPositionCommand(hood, 0.3));
        joystick.b().whileTrue(new ShooterVolSpinCommand(shooter, 0.5));

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
