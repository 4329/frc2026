package frc.robot.model;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HoodLog {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double targetPosition = 0.0;
    public double positionError = 0.0;
    public boolean atTarget = false;
    public boolean motorConnected = false;
}