// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BlinkinColors;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class feederRunCommand extends Command {
  private FeederSubsystem m_feeder;
  private IntakeSubsystem m_intake;
  private BlinkinSubsystem m_blinkin;
  /** Creates a new feederRunCommand. */
  public feederRunCommand(FeederSubsystem feeder, IntakeSubsystem intake, BlinkinSubsystem blinkin) {
    m_feeder = feeder;
    m_intake = intake;
    m_blinkin = blinkin;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.run();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
    if (!m_intake.getNoteSensor()){
      m_blinkin.setToAllianceColor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
