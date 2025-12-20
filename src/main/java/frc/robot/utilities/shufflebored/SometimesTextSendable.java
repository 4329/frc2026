package frc.robot.utilities.shufflebored;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SometimesTextSendable implements Sendable {

    private boolean sometimes;
    private String text;

    public SometimesTextSendable() {
        this.sometimes = false;
        this.text = "";
    }

    public SometimesTextSendable(boolean sometimes, String text) {
        this.sometimes = sometimes;
        this.text = text;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SometimesTextType");
        builder.addBooleanProperty("sometimes", this::getSometimes, this::setSometimes);
        builder.addStringProperty("text", this::getText, this::setText);
    }

    public void setSometimes(boolean sometimes) {
        this.sometimes = sometimes;
    }

    public boolean getSometimes() {
        return sometimes;
    }

    public void setText(String text) {
        this.text = text;
    }

    public String getText() {
        return text;
    }

    public String sometimesGet(String in) {
        return sometimes ? text : in;
    }

    public double sometimesGet(double in) {
        return sometimes ? Double.parseDouble(text) : in;
    }
}
