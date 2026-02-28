package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommand;
// import frc.robot.commands.IntakeAndSpinCommandGroup;
import frc.robot.commands.intakeCommands.IntakeGoToPositionCommand;
import frc.robot.commands.intakeCommands.KickerCommand;
import frc.robot.commands.intakeCommands.SpinMotor13IndefinitelyCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Motor13Spinner;
//import frc.robot.subsystems.SpinMotor44Subsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;
import frc.robot.subsystems.SpinDexterSubsystem;
import frc.robot.subsystems.SpinMotor13Subsystem;
import frc.robot.commands.CommandGroups.TurretSubsystemCommandGroupMax;
import frc.robot.commands.CommandGroups.TurretSubsystemCommandGroupMin;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem.RotateSubsystem;
import frc.robot.subsystems.TurretSubsystem.ShooterSubsystem;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.IntakeAndSpinCommandGroup;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private boolean isFieldCentric = true;
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller
    private final CommandXboxController joystick = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    // private final Motor13Spinner m_crashout = new Motor13Spinner();         // CAN ID 13
    //private final SpinMotor44Subsystem m_crashout13 = new SpinMotor44Subsystem(); // CAN ID 44
    private final SpinDexterSubsystem m_spinDexter = new SpinDexterSubsystem();
    private final SpinMotor13Subsystem m_spin13 = new SpinMotor13Subsystem(13);
    private final KickerSubsystem m_kicker = new KickerSubsystem(60);


    private final HoodSubsystem hood = new HoodSubsystem();

    private final RotateSubsystem turret = new RotateSubsystem();

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private final NEO550ThroughTalonFXSSubsytem spinner = new NEO550ThroughTalonFXSSubsytem(44);

    private final Field2d field = new Field2d();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180));
        SmartDashboard.putData("Field", field);

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("maxShoot", new TurretSubsystemCommandGroupMax(turret, hood, shooter));
    }

    public void robotPeriodic() {
        Logger.recordOutput("Controller/Driver/LeftX", joystick.getLeftX());
        field.setRobotPose(drivetrain.getState().Pose);
        Logger.recordOutput("Drivetrain/Pose", drivetrain.getState().Pose);
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

        // --- INTAKE BINDINGS ---
        // joystick.a().onTrue(
        //     new IntakeAndSpinCommandGroup(m_intake, m_crashout, spinner, m_spinDexter, MaxAngularRate)
        // );

        // joystick.a().onFalse(
        //     new IntakeGoToPositionCommand(m_intake, MaxAngularRate, false)
        // );
        joystick.b().whileTrue(new TurretSubsystemCommandGroupMax(turret, hood, shooter));

        joystick.a().whileTrue(new TurretSubsystemCommandGroupMin(turret, hood, shooter));

            // --- INTAKE BINDINGS ---
    // Run the intake + spin group while button A is held
        joystick.x().whileTrue(
            new IntakeAndSpinCommandGroup(m_intake, m_spin13, m_kicker, m_spinDexter, MaxAngularRate)
        );

    // Return intake to resting position when button A is released
        joystick.x().onFalse(
            new IntakeGoToPositionCommand(m_intake, MaxAngularRate, false)
        );

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