package frc.robot.utilities.shufflebored;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ShuffledPIDController extends PIDController {

    private double output;

    public ShuffledPIDController(double p, double i, double d) {
        super(p, i, d);
    }

    @Override
    public double calculate(double measurement) {
        output = super.calculate(measurement);
        return output;
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        output = super.calculate(measurement, setpoint);
        return output;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("BetterPIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty(
                "iZone",
                this::getIZone,
                (double toSet) -> {
                    try {
                        setIZone(toSet);
                    } catch (IllegalArgumentException e) {
                        MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace());
                    }
                });
        builder.addDoubleProperty("tol", this::getErrorTolerance, this::setTolerance);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("output", () -> output, x -> output = x);
    }
}
