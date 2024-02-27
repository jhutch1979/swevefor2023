// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class intakeRunInCommand extends Command {
  private IntakeSubsystem m_intake;
  private DigitalInput m_noteSensor;
  
  /** Creates a new intakeRunCom. */
  public intakeRunInCommand(IntakeSubsystem intake, DigitalInput noteSensor) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_noteSensor = noteSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_noteSensor.get()){
      end(false);
      return;
    }
    m_intake.runIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_drive.driveWhilePickup(.5,m_joystickY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
