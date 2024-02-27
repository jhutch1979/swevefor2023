// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDrivePickup extends Command {
  private static DriveSubsystem m_drive;
  private DigitalInput NoteIn;
  /** Creates a new AutoDrivePickup. */
  public AutoDrivePickup(DriveSubsystem drive, DigitalInput noteSensor) {
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    NoteIn = noteSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveWhilePickup(.5,-.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drive.drive(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !NoteIn.get();
  }
}
