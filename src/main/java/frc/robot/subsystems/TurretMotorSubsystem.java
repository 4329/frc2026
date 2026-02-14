package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretMotorSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public TurretMotorSubsystem() {
        turretMotor =  new TalonFX(41);

        var motorOutputConfigs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        var Slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        Slot0Configs.withKP(0.1);
        Slot0Configs.withKI(0.0);
        Slot0Configs.withKD(0.0);
        Slot0Configs.withKV(0.12);

        
        var feedbackConfigs = new com.ctre.phoenix6.configs.FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(1.0);
        feedbackConfigs.withRotorToSensorRatio(1.0);
        
        turretMotor.getConfigurator().apply(motorOutputConfigs);
        turretMotor.getConfigurator().apply(Slot0Configs);
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

    public void setPositionWithSpeed(double rotations, double maxVelocityRPS) {
        turretMotor.setControl(positionRequest
            .withPosition(rotations).withVelocity(maxVelocityRPS));  // Limits speed to this value
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
