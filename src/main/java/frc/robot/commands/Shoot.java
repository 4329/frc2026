// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.ExampleShooter;
// import static edu.wpi.first.units.Units.RPM;
// import edu.wpi.first.wpilibj2.command.Command;

// /** A Shoot command that uses the ExampleShooter subsystem. */
// public class Shoot extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ExampleShooter m_subsystem;

//   /**
//    * Creates a new Shoot command.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public Shoot(ExampleShooter subsystem) {
//     m_subsystem = subsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_subsystem.setVelocity(RPM.of(3000)); // Set to 3000 RPM
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_subsystem.setVelocity(RPM.of(0)); // Stop the shooter
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
