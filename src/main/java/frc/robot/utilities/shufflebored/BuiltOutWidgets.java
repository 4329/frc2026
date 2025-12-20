package frc.robot.utilities.shufflebored;

import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public enum BuiltOutWidgets implements WidgetType {
    kRadiableGyro("RadiableGyro"),
    kSometimesText("Sometimes Text"),
    kLoadBar("Loading Bar"),
    kBetterPIDController("BetterPID Controller");

    public String string;

    BuiltOutWidgets(String string) {
        this.string = string;
    }

    @Override
    public String getWidgetName() {
        return string;
    }
}
