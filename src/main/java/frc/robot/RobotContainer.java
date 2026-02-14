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
import frc.robot.commands.intakeCommands.HeeheehahaGoToPositionCommand;
import frc.robot.commands.intakeCommands.VoltageSpinNEO550Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NEO550ThroughTalonFXSSubsytem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private boolean isFieldCentric = true;
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller Mapping
    private final CommandXboxController joystick = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final NEO550ThroughTalonFXSSubsytem spinner = new NEO550ThroughTalonFXSSubsytem();
    private final Intake m_intake = new Intake(); 

    private final Field2d field = new Field2d();
    Map<Command, PathPlannerAuto> autoName = new HashMap<>();

    public RobotContainer() {
        drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180));
        SmartDashboard.putData("Field", field);
        configureBindings();
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

        // --- KRAKEN INTAKE BINDINGS ---
        
    joystick.a().onTrue(new HeeheehahaGoToPositionCommand(m_intake, MaxAngularRate, true));  // Go to 215°
    joystick.a().whileTrue(new VoltageSpinNEO550Command(spinner, 6.0));
    joystick.a().onFalse(new HeeheehahaGoToPositionCommand(m_intake, MaxAngularRate, false)); // Go to 0°
        // --- NEO 550 BINDINGS ---
        joystick.x().whileTrue(new VoltageSpinNEO550Command(spinner, 6.0));
        joystick.y().whileTrue(new VoltageSpinNEO550Command(spinner, -6.0));

        // --- UTILITY BINDINGS ---
        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.povDown().onTrue(Commands.runOnce(() -> isFieldCentric = !isFieldCentric));

        // --- SYSID BINDINGS ---
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);                        
    }

    public Command getAutonomousCommand() {
        return Drive5mAuto.create(drivetrain);
    }

    public String getAutoName(Command command) {
        return autoName.containsKey(command) ? autoName.get(command).getName() : "None";
    }
}