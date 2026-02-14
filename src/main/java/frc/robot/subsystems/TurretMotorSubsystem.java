package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretMotorSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0); // ADD THIS

    public TurretMotorSubsystem() {
        turretMotor = new TalonFX(41);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(0.5);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.06);
        Slot0Configs.withKV(0.12);

        // ADD MOTION MAGIC CONFIGURATION
        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(5); // rotations per second - tune this
        motionMagicConfigs.withMotionMagicAcceleration(20);   // rotations per second^2 - tune this
        motionMagicConfigs.withMotionMagicJerk(200);          // rotations per second^3 - tune this
        
        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);
        
        turretMotor.getConfigurator().apply(motorOutputConfigs);
        turretMotor.getConfigurator().apply(Slot0Configs);
        turretMotor.getConfigurator().apply(motionMagicConfigs); // APPLY THIS
        turretMotor.getConfigurator().apply(feedbackConfigs);

        turretMotor.setPosition(0);

        setDefaultCommand(Commands.run(() -> stop(), this));
    }

    public void spinVoltage(double voltage) {
        turretMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void setPosition(double rotations) {
        turretMotor.setControl(positionRequest.withPosition(rotations));
    }

    public void setVelocity(double rotationsPerSecond) {
        turretMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    // ADD THIS METHOD
    public void setPositionWithVelocity(double rotations) {
        turretMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    public double getPosition() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    @Override
    public void periodic() {
        System.out.println("TURRET ROTATION POSITION: " + getPosition());
        super.periodic();
    }
}







// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class TurretMotorSubsystem extends SubsystemBase {
//     private final TalonFX turretMotor;
//     private final VoltageOut voltageRequest = new VoltageOut(0);
//     private final PositionVoltage positionRequest = new PositionVoltage(0);
//     private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

//     public TurretMotorSubsystem() {
//         turretMotor =  new TalonFX(41);

//         var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
//         motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

//         var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
//         Slot0Configs.withKP(0.1);
//         Slot0Configs.withKI(0.0);
//         Slot0Configs.withKD(0.0);
//         Slot0Configs.withKV(0.12);

        
//         var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
//         feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
//         feedbackConfigs.withSensorToMechanismRatio(1.0);
//         feedbackConfigs.withRotorToSensorRatio(1.0);
        
//         turretMotor.getConfigurator().apply(motorOutputConfigs);
//         turretMotor.getConfigurator().apply(Slot0Configs);
//         turretMotor.getConfigurator().apply(feedbackConfigs);

//         turretMotor.setPosition(0);

//         setDefaultCommand(Commands.run(() -> stop(), this));
//     }

//     public void spinVoltage(double voltage) {
//         turretMotor.setControl(voltageRequest.withOutput(voltage));
//     }

//     public void setPosition(double rotations) {
//         turretMotor.setControl(positionRequest.withPosition(rotations));
//     }

//     public void setVelocity(double rotationsPerSecond) {
//         turretMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
//     }

//     public double getPosition() {
//         return turretMotor.getPosition().getValueAsDouble();
//     }

//     public void stop() {
//         turretMotor.stopMotor();
//     }

//     @Override
//     public void periodic() {
//         System.out.println("TURRET ROTATION POSITION: " + getPosition());
//         super.periodic();
//     }

    
// }
