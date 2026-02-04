package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class FollowAprilTagCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    
    private final PIDController forwardController;
    private final PIDController rotationController;
    
    public FollowAprilTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        // Create PID controllers for robot movement
        this.forwardController = new PIDController(
            VisionConstants.FOLLOW_KP_TRANSLATION, 0, 0
        );
        this.rotationController = new PIDController(
            VisionConstants.FOLLOW_KP_ROTATION, 0, 0
        );
        
        // Set rotation controller to be continuous (wraps around at +/- 180°)
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(VisionConstants.ANGLE_TOLERANCE);
        
        addRequirements(vision, drivetrain);
    }
    
    @Override
    public void initialize() {
        // Reset PID controllers
        forwardController.reset();
        rotationController.reset();
        System.out.println("FollowAprilTag command initialized");
    }
    
    @Override
    public void execute() {
        // Check if we have a target
        boolean hasTarget = vision.hasTarget();
        SmartDashboard.putBoolean("FollowTag/HasTarget", hasTarget);
        
        if (!hasTarget) {
            // No target, stop the robot
            System.out.println("No target - stopping");
            drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake());
            return;
        }
        
        // Get target information
        double tx = vision.getTargetTX(); // Horizontal offset in degrees
        double distance = vision.getTargetDistance(); // Distance to tag in meters
        
        // Calculate speeds using PID controllers
        double forwardSpeed = forwardController.calculate(distance, VisionConstants.FOLLOW_DISTANCE_METERS);
        double rotationSpeed = rotationController.calculate(tx, 0); // We want tx to be 0 (centered)
        
        // Print raw PID outputs
        System.out.printf("TX: %.2f, Dist: %.2f, RawFwd: %.3f, RawRot: %.3f%n", 
            tx, distance, forwardSpeed, rotationSpeed);
        
        // Very aggressive clamping for testing
        forwardSpeed = MathUtil.clamp(forwardSpeed, -0.3, 0.3); // Max 0.3 m/s
        rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5); // Max 0.5 rad/s
        
        // Print clamped outputs
        System.out.printf("  Clamped - Fwd: %.3f, Rot: %.3f%n", forwardSpeed, rotationSpeed);
        
        // Put on dashboard
        SmartDashboard.putNumber("FollowTag/TX", tx);
        SmartDashboard.putNumber("FollowTag/Distance", distance);
        SmartDashboard.putNumber("FollowTag/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("FollowTag/RotationSpeed", rotationSpeed);
        
        // Apply speeds to drivetrain using robot-centric control
        var request = new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
            .withVelocityX(forwardSpeed)
            .withVelocityY(0) // No strafe
            .withRotationalRate(rotationSpeed);
        
        drivetrain.setControl(request);
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("FollowAprilTag command ended");
        // Stop the robot when command ends
        drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake());
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}