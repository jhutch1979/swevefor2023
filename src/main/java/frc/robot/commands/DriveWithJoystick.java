// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithJoystick extends Command {
  private Joystick m_Joystick;
  private DriveSubsystem m_chassis;
  /** Creates a new SriveWithJoystick. */
  public DriveWithJoystick(Joystick joystick, DriveSubsystem chassis) {
    m_Joystick = joystick;
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double scale;// = -m_Joystick.getThrottle();
    //scale += 1;
    //scale *= 0.15;
    scale = .5;
    SmartDashboard.putNumber("speed scale", scale);
    if(m_Joystick.getRawButton(2)){
      m_chassis.driveWhilePickup(
                    scale,
                    -m_Joystick.getY());
    } else if(m_Joystick.getRawButton(1)){
      m_chassis.driveWhileShhoting(
        scale, -m_Joystick.getY(), -m_Joystick.getX(),m_Joystick.getZ());
    }else {
      m_chassis.drive(
      scale,
                    -m_Joystick.getY()*scale,
                    -m_Joystick.getX()*scale,
                    -m_Joystick.getZ()*scale);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
