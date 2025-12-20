package frc.robot.utilities;

import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;

public enum CenterDistance {
    INITIAL(DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER + 0.3, 0.1, 0.1),
    SCORING(DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER, 0.05, 0.04),
    SCORING_SMALL(0.5, 0.05, 0.04),
    PORCESSOR(0.67, 0.1, 0.1);

    private double zDist;
    private double translationTolerance;
    private double rotationTolerance;

    CenterDistance(double zDist, double translationTolerance, double rotationTolerance) {
        this.zDist = zDist;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;
    }

    public double getzDist() {
        return zDist;
    }

    public double getTranslationTolerance() {
        return translationTolerance;
    }

    public double getRotationTolerance() {
        return rotationTolerance;
    }
}
