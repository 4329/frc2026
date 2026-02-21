package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinMotor13thingy;

public class SpinMotor13Command extends Command {
    private final SpinMotor13thingy m_crashout;
    private final double m_voltage;
    private final boolean m_spin;

    public SpinMotor13Command(SpinMotor13thingy crashout, double voltage, boolean spin) {
        m_crashout = crashout;
        m_voltage = voltage;
        m_spin = spin;
        addRequirements(crashout);
    }

    @Override
    public void execute() {
        if (m_spin) {
            m_crashout.spinVoltage(m_voltage);
        } else {
            m_crashout.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_crashout.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}