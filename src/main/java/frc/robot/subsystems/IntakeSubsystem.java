// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static final CANSparkMax m_intakeMotor = new CANSparkMax(23, MotorType.kBrushless);
  private static DigitalInput IsNoteIN = new DigitalInput(0);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.setInverted(false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("note in", getNoteSensor());

  }

  public void runIn(){
    m_intakeMotor.set(1);
  }
  public void runOut(){
    m_intakeMotor.set(-1);
  }

  public void stop(){
    m_intakeMotor.set(0);
  }

  public boolean getNoteSensor(){
   return IsNoteIN.get();
  }
}
