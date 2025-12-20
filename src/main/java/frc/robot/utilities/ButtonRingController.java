package frc.robot.utilities;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.model.ButtonRingLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ButtonRingController extends CommandGenericHID implements LoggedSubsystem, Sendable {
    private int level;

    private double xOffset;
    private int button;
    private int tagID;
    private double OFFSET_AMOUNT = Units.inchesToMeters(13) / 2;
    GenericEntry bsdf = Shuffleboard.getTab("Asdf").add("dst", OFFSET_AMOUNT).getEntry();

    ButtonRingLogAutoLogged buttonRingLogAutoLogged;

    public ButtonRingController(int port) {
        super(port);

        new UnInstantCommand(
                        "SetButtonRingLevel",
                        () -> {
                            if (getRawAxis(0) == 1) level = 2;
                            else if (getRawAxis(0) == -1) level = 1;
                            else if (getRawAxis(1) == 1) level = 3;
                            else if (getRawAxis(1) == -1) level = 4;
                            else level = 0;
                        })
                .repeatedlyLog()
                .ignoringDisableLog(true)
                .schedule();

        new UnInstantCommand(
                        "TryButtons",
                        () -> {
                            boolean no = false;
                            for (int i = 1; i <= 12; i++) {
                                if (!button(i).getAsBoolean()) continue;
                                no = true;
                                button = i;

                                OFFSET_AMOUNT = bsdf.getDouble(0); // WIP

                                xOffset = OFFSET_AMOUNT * (i % 2 == 0 ? 1 : -1);
                                tagID = AprilTagUtil.getReef((i % 12) / 2);
                            }

                            if (!no) {
                                button = 0;
                                xOffset = 0;
                                tagID = 0;
                            }
                        })
                .repeatedlyLog()
                .ignoringDisableLog(true)
                .schedule();

        buttonRingLogAutoLogged = new ButtonRingLogAutoLogged();
    }

    @Override
    public LoggableInputs log() {
        buttonRingLogAutoLogged.level = level;
        buttonRingLogAutoLogged.xOffset = xOffset;
        buttonRingLogAutoLogged.tagID = tagID;
        return buttonRingLogAutoLogged;
    }

    public double getxOffset() {
        return xOffset;
    }

    public int getLevel() {
        return level;
    }

    public int getTagID() {
        return tagID;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Octagon");
        builder.addIntegerProperty("button", () -> button, (a) -> button = (int) a);
        builder.addIntegerProperty("level", this::getLevel, (a) -> level = (int) a);
    }
}
