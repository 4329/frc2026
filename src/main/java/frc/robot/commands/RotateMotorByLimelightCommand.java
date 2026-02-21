package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class RotateMotorByLimelightCommand extends Command {

    private static final int MOTOR_ID = 43;
    private static final int TARGET_TAG_ID = 4;
    private static final double MAX_OUTPUT = 0.4; // Max duty cycle (0 to 1). Tune this.

    private final TalonFX motor;
    private final PIDController rotationPID;
    private final DutyCycleOut driveRequest = new DutyCycleOut(0);

    public RotateMotorByLimelightCommand() {
        this.motor = new TalonFX(MOTOR_ID);

        // P = how aggressively to correct. I = fix persistent error. D = dampen oscillation.
        // TX is in degrees (-29.8 to 29.8 for LL4), so these are tuned to that scale.
        this.rotationPID = new PIDController(0.03, 0.0, 0.001);
        this.rotationPID.setTolerance(1.5); // degrees — within this, consider it centered
        this.rotationPID.setSetpoint(0.0);  // we always want TX = 0 (tag centered)
    }

    @Override
    public void initialize() {
        rotationPID.reset();
    }

    @Override
    public void execute() {
        String ll = VisionConstants.LIMELIGHT_TURRET_NAME;

        boolean hasTarget = LimelightHelpers.getTV(ll);
        double tid = LimelightHelpers.getFiducialID(ll);

        if (!hasTarget || (int) tid != TARGET_TAG_ID) {
            motor.setControl(driveRequest.withOutput(0));
            return;
        }

        double tx = LimelightHelpers.getTX(ll);

        // PID calculates how much to rotate based on how far off-center TX is.
        // TX > 0 means tag is to the right, so output will be positive -> rotate right.
        // TX < 0 means tag is to the left, so output will be negative -> rotate left.
        double output = rotationPID.calculate(tx);
        output = MathUtil.clamp(output, -MAX_OUTPUT, MAX_OUTPUT);

        motor.setControl(driveRequest.withOutput(output));
    }

    @Override
    public void end(boolean interrupted) {
        motor.setControl(driveRequest.withOutput(0));
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}