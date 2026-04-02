package frc.robot.model;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakePivotLog {
public double targetPosition = 0.0;
public double positionRotations = 0.0;
public double appliedVolts = 0.0; 
public double supplyCurrentAmps = 0.0;
public double tempCelsius = 0.0;
public double positionError = 0.0;
public boolean atTarget = false;
public boolean motorConnected = false;  
}
