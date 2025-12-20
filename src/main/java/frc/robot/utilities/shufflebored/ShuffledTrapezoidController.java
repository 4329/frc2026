package frc.robot.utilities.shufflebored;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ShuffledTrapezoidController extends ProfiledPIDController {

    private double output;

    public ShuffledTrapezoidController(
            double p, double i, double d, TrapezoidProfile.Constraints constraints) {
        super(p, i, d, constraints);
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
        builder.addDoubleProperty("tol", this::getPositionTolerance, this::setTolerance);
        builder.addDoubleProperty("setpoint", () -> getGoal().position, this::setGoal);
        builder.addDoubleProperty("output", () -> output, x -> output = x);
        builder.addDoubleProperty(
                "speed",
                () -> getConstraints().maxVelocity,
                x -> setConstraints(new TrapezoidProfile.Constraints(x, getConstraints().maxAcceleration)));
        builder.addDoubleProperty(
                "accel",
                () -> getConstraints().maxAcceleration,
                x -> setConstraints(new TrapezoidProfile.Constraints(getConstraints().maxAcceleration, x)));
    }
}
