package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeGoToPositionCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final double m_maxAngularRate;
    private final boolean m_goTo215; // true = go to 215°, false = go to 0°
    
    public IntakeGoToPositionCommand(IntakeSubsystem intake, double maxAngularRate, boolean goTo215) {
        this.m_intake = intake;
        this.m_maxAngularRate = maxAngularRate;
        this.m_goTo215 = goTo215;
        addRequirements(intake);
            }
        
            @Override
    public void initialize() {
        if (m_goTo215) {
            m_intake.goTo215();
        } else {
            m_intake.goTo0();
        }
    }
    
    @Override
    public boolean isFinished() {
        // Get current position
        double currentPos = m_intake.getPosition();
        double targetPos = m_goTo215 ? m_intake.getPos215() : m_intake.getPos0();
        
        // Check if we're close enough (within 0.01 rotations, adjust tolerance as needed)
        return Math.abs(currentPos - targetPos) < 0.01;
    }
}