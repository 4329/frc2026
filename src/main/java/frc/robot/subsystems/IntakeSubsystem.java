package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SparkIDs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor = new SparkMax(SparkIDs.intakePivot, MotorType.kBrushless);
    private final SparkMax rollerMotor = new SparkMax(SparkIDs.intakeWheel, MotorType.kBrushless);
    
    private final SparkClosedLoopController pivotPID = pivotMotor.getClosedLoopController();

    public IntakeSubsystem() {
        // Pivot Configuration (2026 API)
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.closedLoop
            .p(IntakeConstants.kP)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .maxMotion
                .maxVelocity(IntakeConstants.kMaxVel)
                .maxAcceleration(IntakeConstants.kMaxAcc)
                .allowedClosedLoopError(0.5);
        
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Roller Configuration
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(30);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runRoller(double speed) {
        rollerMotor.set(speed);
    }

    public void runPivot(double speed) {
        pivotMotor.set(speed);
    }

    public void setPivotPosition(double rotations) {
        pivotPID.setReference(rotations, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    public void runHoming() {
        pivotMotor.set(-0.12);
        if (pivotMotor.getOutputCurrent() > IntakeConstants.kHomingCurrent) {
            pivotMotor.set(0);
            pivotMotor.getEncoder().setPosition(0);
        }
    }
}