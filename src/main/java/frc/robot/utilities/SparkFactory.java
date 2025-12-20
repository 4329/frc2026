package frc.robot.utilities;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkFactory {

    public static SparkMax createSparkMax(int id) {

        return createSparkMax(id, false);
    }

    public static SparkMax createSparkMax(int id, Boolean flipSparkMax) {
        return createSparkMax(id, flipSparkMax, false);
    }

    public static SparkMax createSparkMax(int id, Boolean flipSparkMax, boolean resetted) {
        // something to the effect of we experimented with burn flash and reset to
        // factory defaults, and it caused the drive motors to go bonkers
        SparkMax canToMake = new SparkMax(id, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
                .idleMode(IdleMode.kBrake)
                .disableFollowerMode()
                .inverted(false);

        canToMake.getEncoder().setPosition(0);

        canToMake.configure(
                config,
                resetted ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        return canToMake;
    }
}
