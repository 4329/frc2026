package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagUtil {
    public static final int[] blueTags = new int[] {19, 20, 21, 22, 17, 18};
    public static final int[] redTags = new int[] {6, 11, 10, 9, 8, 7};

    public static int getReef(int num) {
        if (Alliance.Red.equals(DriverStation.getAlliance().orElse(null))) return redTags[num];
        else return blueTags[num];
    }

    public static int getPorcessor() {
        return Alliance.Red.equals(DriverStation.getAlliance().orElse(null)) ? 3 : 16;
    }
}
