package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CompCommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class FollowAprilTagCommand extends Command {
    private final VisionSubsystem vision;
    private final CompCommandSwerveDrivetrain drivetrain;
    
    private final PIDController forwardController;
    private final PIDController rotationController;
    
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public FollowAprilTagCommand(VisionSubsystem vision, CompCommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        this.forwardController = drivetrain.getVisionForwardController();
        this.rotationController = drivetrain.getVisionRotationController();
        
        addRequirements(vision, drivetrain);
    }
    
    @Override
    public void initialize() {
        drivetrain.resetVisionControllers();
    }
    
    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            
            drivetrain.setControl(
                driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
            return;
        }
        
        double tx = vision.getTargetTX();
        double distance = vision.getTargetDistance();
        
        double forwardSpeed = -forwardController.calculate(distance, VisionConstants.FOLLOW_DISTANCE_METERS);
        double rotationSpeed = -rotationController.calculate(tx, 0);
        
        if (Math.abs(tx) < 1.5) {
            rotationSpeed = 0.0;
        }
        
        forwardSpeed = MathUtil.clamp(forwardSpeed, -2.0, 2.0);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -4.0, 4.0);
        
        System.out.printf("TX: %.2f, Dist: %.2f, Fwd: %.3f, Rot: %.3f%n", 
            tx, distance, forwardSpeed, rotationSpeed);
        
        SmartDashboard.putNumber("FollowTag/TX", tx);
        SmartDashboard.putNumber("FollowTag/Distance", distance);
        SmartDashboard.putNumber("FollowTag/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("FollowTag/RotationSpeed", rotationSpeed);
        
        drivetrain.setControl(
            driveRequest
                .withVelocityX(forwardSpeed)
                .withVelocityY(0.0)
                .withRotationalRate(rotationSpeed)
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}