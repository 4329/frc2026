package frc.robot.utilities;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * motor logger for logging motor data to SmartDashboard. Call the appropriate log method inside your subsystem's periodic() method.
 * Currently supports TalonFX and SparkMax/SparkFlex (NEO/NEO Vortex). Add more methods for other motor types as needed.
 * thank claude for it
*/
public class MotorLogger {

    public static void logTalonFX(TalonFX motor, String name) {
            

        SmartDashboard.putNumber(name + "/Output Voltage (V)",
                motor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber(name + "/Supply Current (A)",
                motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(name + "/Stator Current (A)",
                motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber(name + "/Velocity (RPS)",
                motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(name + "/Position (Rotations)",
                motor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber(name + "/Temperature (C)",
                motor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putString(name + "/Control Mode",
                motor.getControlMode().toString());

        SmartDashboard.putBoolean(name + "/Has Fault",
                motor.getFault_Hardware().getValue()
                || motor.getFault_Undervoltage().getValue()
                || motor.getFault_BootDuringEnable().getValue());
    }

   
    public static void logSpark(SparkBase motor, RelativeEncoder encoder, String name) {
        
        SmartDashboard.putNumber(name + "/Applied Output (%)",
                motor.getAppliedOutput());
        SmartDashboard.putNumber(name + "/Bus Voltage (V)",
                motor.getBusVoltage());

        
        SmartDashboard.putNumber(name + "/Output Current (A)",
                motor.getOutputCurrent());

        
        SmartDashboard.putNumber(name + "/Velocity (RPM)",
                encoder.getVelocity());
        SmartDashboard.putNumber(name + "/Position (Rotations)",
                encoder.getPosition());

        
        SmartDashboard.putNumber(name + "/Temperature (C)",
                motor.getMotorTemperature());
        SmartDashboard.putBoolean(name + "/Has Fault",
                motor.hasActiveFault());
    }
}