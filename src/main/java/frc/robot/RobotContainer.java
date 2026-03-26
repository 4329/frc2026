package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AimAtHubCommand;
import frc.robot.commands.DoAFunctionalCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.CommandGroups.FullTurretCommandGroup;
import frc.robot.commands.CommandGroups.HoodToZeroCommandGroup;
import frc.robot.commands.CommandGroups.IntakeInCommandGroup;
import frc.robot.commands.CommandGroups.IntakeOutCommandGroup;
import frc.robot.commands.CommandGroups.SpindexerAndKickerCommandGroup;
import frc.robot.commands.CommandGroups.TurretCommandGroup;
import frc.robot.subsystems.SpindexterSubsystem;
import frc.robot.commands.IntakeCommands.IntakePivotCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.commands.IntakeCommands.IntakeZeroCommand;
import frc.robot.commands.TurretCommands.HoodCommands.HoodZeroCommand;
import frc.robot.commands.TurretCommands.HoodCommands.ManualHoodCommand;
import frc.robot.commands.TurretCommands.HoodCommands.SetHoodPositionCommand;
import frc.robot.commands.TurretCommands.RotationCommands.TurretPositionCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVelocityCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVolSpinCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private boolean isFieldCentric = true;
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller
    private final CommandXboxController joystick = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
    private final CommandXboxController operator = new CommandXboxController(Constants.OIConstants.kOperatorControllerPort);
    private final CommandXboxController functional = new CommandXboxController(Constants.OIConstants.kFunctionalControllerPort);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SpindexterSubsystem spindexter = new SpindexterSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    

    private final HoodSubsystem hood = new HoodSubsystem();
    private final RotateSubsystem turretRotate = new RotateSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private final VisionSubsystem vision = new VisionSubsystem(drivetrain, turretRotate);

    private final IntakePivotSubsystem pivot = new IntakePivotSubsystem();
    private final IntakeSpinSubsystem spin = new IntakeSpinSubsystem();

    private final Field2d field = new Field2d();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        SmartDashboard.putData("Field", field);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();

        SmartDashboard.putNumber("Tuning/ShooterSpeed", 70.0);
        SmartDashboard.putNumber("Tuning/HoodAngle", 2.0);
        SmartDashboard.putNumber("Tuning/SpindexerSpeed", 45.0);

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        SmartDashboard.putData("PDH", pdh);
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle",
                    () -> drivetrain.getState().ModuleStates[0].angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity",
                    () -> drivetrain.getState().ModuleStates[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle",
                    () -> drivetrain.getState().ModuleStates[1].angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity",
                    () -> drivetrain.getState().ModuleStates[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle",
                    () -> drivetrain.getState().ModuleStates[2].angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity",
                    () -> drivetrain.getState().ModuleStates[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle",
                    () -> drivetrain.getState().ModuleStates[3].angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity",
                    () -> drivetrain.getState().ModuleStates[3].speedMetersPerSecond, null);


                builder.addDoubleProperty("Robot Angle", () -> drivetrain.getState().Pose.getRotation().getRadians(), null);
            }
        });
    }

    public void robotPeriodic() {
        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false);

        var pose = drivetrain.getState().Pose;

        if (isRed) {
            field.setRobotPose(new Pose2d(
                16.5412 - pose.getX(),
                8.0137  - pose.getY(),
                pose.getRotation().plus(Rotation2d.fromDegrees(180))
            ));
        } else {
            field.setRobotPose(pose);
        }

        Logger.recordOutput("Controller/Driver/LeftX", joystick.getLeftX());
        field.setRobotPose(drivetrain.getState().Pose);
        Logger.recordOutput("Drivetrain/Pose", drivetrain.getState().Pose);
        
        Logger.recordOutput("Drivetrain/Speeds", new double[] {
            drivetrain.getState().Speeds.vxMetersPerSecond,
            drivetrain.getState().Speeds.vyMetersPerSecond,
            drivetrain.getState().Speeds.omegaRadiansPerSecond
        });

        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putString("Drive Mode", isFieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC");
        SmartDashboard.putBoolean("Field Oriented", isFieldCentric);
    }

    private void configureBindings() {
        // --- SWERVE DRIVE COMMAND ---
        drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain, 
            joystick::getLeftX, 
            joystick::getLeftY, 
            joystick::getRightX, 
            () -> isFieldCentric, 
            MaxSpeed, 
            MaxAngularRate));

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        // joystick.a().whileTrue(Commands.run(() -> shooter.setVelocity(SmartDashboard.getNumber("Tuning/ShooterSpeed", 70.0)), shooter).finallyDo(() -> shooter.stop()));
        // joystick.b().onTrue(new SetHoodPositionCommand(hood, () -> SmartDashboard.getNumber("Tuning/HoodAngle", 2.0)));
        // joystick.x().whileTrue(Commands.run(() -> spindexter.setVelocity(SmartDashboard.getNumber("Tuning/SpindexerSpeed", 45)), spindexter).finallyDo(() -> spindexter.stop()));
        // joystick.y().onTrue(new HoodToZeroCommandGroup(hood));
        // joystick.povRight().onTrue(new SetHoodPositionCommand(hood, 4.5));



        joystick.rightTrigger().whileTrue(new SpindexerAndKickerCommandGroup(spindexter, kicker));
        joystick.leftTrigger().whileTrue(new FullTurretCommandGroup(spin, hood, shooter, vision, drivetrain, joystick::getLeftX, joystick::getLeftY));
        joystick.leftTrigger().onFalse(new HoodToZeroCommandGroup(hood));

        joystick.leftBumper().onTrue(new IntakeOutCommandGroup(pivot, spin));
        joystick.leftBumper().onFalse(Commands.runOnce(() -> spin.stop(), spin));
        joystick.leftBumper().onFalse(new IntakePivotCommand(pivot, 0.0));
        
        joystick.rightBumper().whileTrue(new IntakeSpinCommand(spin, -60));

        joystick.povLeft().whileTrue(new TurretCommandGroup(hood, shooter, spindexter, vision));
        joystick.povRight().whileTrue(new AimAtHubCommand(drivetrain, joystick::getLeftX, joystick::getLeftY));
        
        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.povDown().onTrue(Commands.runOnce(() -> isFieldCentric = !isFieldCentric));


        operator.leftTrigger().onTrue(new IntakeInCommandGroup(pivot, spin));
        operator.rightTrigger().onTrue(new HoodToZeroCommandGroup(hood));


        functional.a().onTrue(new DoAFunctionalCommand(drivetrain, functional.getHID(), pivot, spin, spindexter, kicker, hood, shooter));


        drivetrain.registerTelemetry(logger::telemeterize);                        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}