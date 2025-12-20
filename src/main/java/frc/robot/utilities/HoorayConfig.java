package frc.robot.utilities;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import org.apache.commons.io.FileUtils;

public class HoorayConfig {

    private static Config config;
    private static GenericEntry configEntry;

    public static Config gimmeConfig() {

        if (config == null) {
            configEntry = Shuffleboard.getTab("Config").add("Current Config", "").getEntry();

            if (Constants.robotMode == Constants.Mode.REAL) {
                File configFile = findConfig();
                config = parseConfig(configFile);
            } else {
                config = new Config();
            }
        }

        return config;
    }

    private static Config parseConfig(File configFile) {

        try {

            String json = FileUtils.readFileToString(configFile, Charset.defaultCharset());

            ObjectMapper objectMapper = new ObjectMapper();
            return objectMapper.readValue(json, Config.class);
        } catch (IOException e) {

            e.printStackTrace();
            throw new RuntimeException("Couldn't Read Config File " + configFile, e);
        }
    }

    private static File findConfig() {

        if (new File("/home/lvuser/proto").exists()) {

            configEntry.setString("Proto");
            return new File("/home/lvuser/deploy/protoConfig.json");
        } else if (new File("/home/lvuser/dev").exists()) {

            configEntry.setString("Dev");
            return new File("/home/lvuser/deploy/devConfig.json");
        } else {

            configEntry.setString("Comp");
            return new File("/home/lvuser/deploy/compConfig.json");
        }
    }
}
