// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private static final CANSparkMax climberMotor = new CANSparkMax(24, MotorType.kBrushless);
  /** Creates a new FeederSubsystem. */
  public ClimberSubsystem() {
    climberMotor.setInverted(true);
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(){
    climberMotor.set(.8);
  }

  public void reverse(){
    climberMotor.set(-.6);
  }

  public void stop(){
    climberMotor.set(0);
  }
}
