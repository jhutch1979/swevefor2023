// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRunCommand extends Command {
  private static ClimberSubsystem m_climber;
  private static Joystick m_joystick;
  /** Creates a new ClimberRunCommand. */
  public ClimberRunCommand(ClimberSubsystem climber, Joystick joystick) 
  {
    m_climber = climber;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.run(m_joystick.getRawButton(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
