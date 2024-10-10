// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeRunInCommand extends Command {
  private IntakeSubsystem m_intake;
  private BlinkinSubsystem m_blinkin;
  //private DigitalInput m_noteSensor;
  //private static DigitalInput noteSensor = new DigitalInput(0);
  
  /** Creates a new intakeRunCom. */
  public intakeRunInCommand(IntakeSubsystem intake, BlinkinSubsystem blinkin) {
    m_intake = intake;
    m_blinkin = blinkin;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //if (!m_intake.getNoteSensor().get()){
    //  end(false);
    //  return;
    //}
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
    m_intake.stop();
    m_blinkin.changePattern(Constants.BlinkinColors.noteIn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.getNoteSensor();
  }

  
}
