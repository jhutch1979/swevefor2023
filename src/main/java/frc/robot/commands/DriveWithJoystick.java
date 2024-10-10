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
  private Joystick m_driverStationButtons;
  private DriveSubsystem m_chassis;
  /** Creates a new SriveWithJoystick. */
  public DriveWithJoystick(Joystick joystick, DriveSubsystem chassis, Joystick DriverStaionButtons) {
    m_Joystick = joystick;
    m_driverStationButtons = DriverStaionButtons;
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
    double scale = -m_Joystick.getRawAxis(2);// was getThrottle();
    scale += 1;
    scale *= 0.13;
    scale += .75;
    if (m_driverStationButtons.getRawButton(9)){
      scale = .25;
    }
    SmartDashboard.putNumber("speed scale", scale);
    if(m_Joystick.getRawButton(9)){
      m_chassis.driveWhilePickup(
                    .5,
                    -m_Joystick.getRawAxis(1)*scale);
    } else if(m_Joystick.getRawButton(2)){
      m_chassis.driveWhileShhoting(
        scale, -m_Joystick.getRawAxis(1)*scale, -m_Joystick.getRawAxis(0)*scale,-m_Joystick.getRawAxis(5)*scale);
    }else {
      m_chassis.drive(
      scale,
                    -m_Joystick.getRawAxis(1)*scale,
                    -m_Joystick.getRawAxis(0)*scale,
                    -m_Joystick.getRawAxis(5)*scale);
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
