package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Motor13Spinner;
//import frc.robot.subsystems.SpinMotor44Subsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;
import frc.robot.commands.KickerSpinCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.subsystems.SpinDexterSubsystem;
import frc.robot.commands.CommandGroups.IntakeSubsystemCommandGroup;
import frc.robot.commands.CommandGroups.SDandKCommandGroup;
import frc.robot.commands.CommandGroups.SpindexerAndKickerAndShooterCommandGroup;
import frc.robot.commands.CommandGroups.TurretSubsystemCommandGroupMax;
import frc.robot.commands.CommandGroups.TurretSubsystemCommandGroupMin;
import frc.robot.commands.IntakeCommands.IntakePivotCommand;
import frc.robot.commands.IntakeCommands.IntakeSpinCommand;
import frc.robot.commands.TurretCommands.HoodCommands.AmpZeroing;
import frc.robot.commands.TurretCommands.HoodCommands.SetHoodPositionCommand;
import frc.robot.commands.TurretCommands.RotationCommands.TurretPositionCommand;
import frc.robot.commands.TurretCommands.ShooterCommands.ShooterVelocityCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;
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

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final Motor13Spinner m_crashout = new Motor13Spinner();         // CAN ID 13
    //private final SpinMotor44Subsystem m_crashout13 = new SpinMotor44Subsystem(); // CAN ID 44
    private final SpinDexterSubsystem m_spinDexter = new SpinDexterSubsystem();
    private final KickerSubsystem m_kicker = new KickerSubsystem(16);
    

    private final HoodSubsystem hood = new HoodSubsystem();

    private final RotateSubsystem turret = new RotateSubsystem();

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private final NEO550ThroughTalonFXSSubsytem spinner = new NEO550ThroughTalonFXSSubsytem(44);

    private final IntakePivotSubsystem pivot = new IntakePivotSubsystem();
    private final IntakeSpinSubsystem spin = new IntakeSpinSubsystem();

    private final Field2d field = new Field2d();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180));
        SmartDashboard.putData("Field", field);

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();

        SmartDashboard.putNumber("Tuning/ShooterSpeed", 50.0);
        SmartDashboard.putNumber("Tuning/HoodAngle", 0.1);

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

    private void registerNamedCommands() {
        NamedCommands.registerCommand("maxShoot", new TurretSubsystemCommandGroupMax(turret, hood, shooter));
    }

    public void robotPeriodic() {
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

        // joystick.a().whileTrue(new ShooterVelocityCommand(shooter, SmartDashboard.getNumber("Tuning/ShooterSpeed", 50)));
        // joystick.b().whileTrue(new SetHoodPositionComman
        joystick.b().whileTrue(new ShooterVelocityCommand(shooter, -60));

           joystick.x().whileTrue(new IntakeSpinCommand(spin, 60));
           
           joystick.y().whileTrue(new SpindexerAndKickerAndShooterCommandGroup(m_kicker, m_spinDexter, shooter));
           
           joystick.povLeft().whileTrue(new SetHoodPositionCommand(hood, 3));
           joystick.povRight().whileTrue(new SetHoodPositionCommand(hood, 6));
                      
           joystick.rightBumper().whileTrue(new KickerSpinCommand(m_kicker, -200));
           joystick.rightTrigger().onTrue(new AmpZeroing(hood));
           joystick.leftBumper().whileTrue(new SpindexerCommand(m_spinDexter, -75));

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


        // --- UTILITY BINDINGS ---
        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.povDown().onTrue(Commands.runOnce(() -> isFieldCentric = !isFieldCentric));

        // --- SYSID BINDINGS ---
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);                        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}