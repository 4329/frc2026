package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class FollowAprilTagCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    
    private final PIDController forwardController;
    private final PIDController rotationController;
    
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private int framesWithoutTarget = 0;
    private static final int MAX_FRAMES_WITHOUT_TARGET = 10; // 0.2 seconds at 50Hz
    
    public FollowAprilTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        // Increased gains for faster movement
        this.forwardController = new PIDController(1.2, 0, 0);  // Increased for faster translation
        this.rotationController = new PIDController(0.15, 0, 0); // Increased for much faster rotation
        
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(2.0);
        
        addRequirements(vision, drivetrain);
    }
    
    @Override
    public void initialize() {
        forwardController.reset();
        rotationController.reset();
        framesWithoutTarget = 0;
        System.out.println("FollowAprilTag - STARTED");
    }
    
    @Override
    public void execute() {
        // CRITICAL: If no target, STOP IMMEDIATELY
        if (!vision.hasTarget()) {
            framesWithoutTarget++;
            System.out.println("No target seen - STOPPING - frames without: " + framesWithoutTarget);
            
            // Stop all movement immediately
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0);
            drivetrain.setControl(driveRequest);
            return;
        }
        
        // Reset counter when we see the target
        framesWithoutTarget = 0;
        
        double tx = vision.getTargetTX();
        double distance = vision.getTargetDistance();
        
        // Calculate speeds
        double forwardSpeed = -forwardController.calculate(distance, VisionConstants.FOLLOW_DISTANCE_METERS);
        double rotationSpeed = -rotationController.calculate(tx, 0);
        
        // Smaller deadband for rotation
        if (Math.abs(tx) < 1.5) {
            rotationSpeed = 0.0;
        }
        
        // Increased speed limits for faster movement
        forwardSpeed = MathUtil.clamp(forwardSpeed, -2.0, 2.0);   // Increased from 1.5
        rotationSpeed = MathUtil.clamp(rotationSpeed, -4.0, 4.0); // Increased from 2.0 for much faster rotation
        
        System.out.printf("TX: %.2f, Dist: %.2f, Fwd: %.3f, Rot: %.3f%n", 
            tx, distance, forwardSpeed, rotationSpeed);
        
        SmartDashboard.putNumber("FollowTag/TX", tx);
        SmartDashboard.putNumber("FollowTag/Distance", distance);
        SmartDashboard.putNumber("FollowTag/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("FollowTag/RotationSpeed", rotationSpeed);
        
        driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(0.0)
            .withRotationalRate(rotationSpeed);
        
        drivetrain.setControl(driveRequest);
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("FollowAprilTag ended - interrupted: " + interrupted);
        
        // Stop all movement when command ends
        driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);
        drivetrain.setControl(driveRequest);
    }
    
    @Override
    public boolean isFinished() {
        // End command if we haven't seen target for too long
        return framesWithoutTarget >= MAX_FRAMES_WITHOUT_TARGET;
    }
}