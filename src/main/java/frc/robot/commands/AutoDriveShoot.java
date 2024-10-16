// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveShoot extends Command {
  private static DriveSubsystem m_drive;
  
  
  /** Creates a new AutoDrivePickup. */
  public AutoDriveShoot(DriveSubsystem drive){//, DigitalInput noteSensor) {
    m_drive = drive;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    //NoteIn = noteSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveWhileShhoting(1, 0,0,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drive.drive(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
