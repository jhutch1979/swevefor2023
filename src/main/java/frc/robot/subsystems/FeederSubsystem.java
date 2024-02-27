// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  private static final CANSparkMax feederMotor = new CANSparkMax(22, MotorType.kBrushless);
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    feederMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(){
    feederMotor.set(1);
  }

  public void reverse(){
    feederMotor.set(-.6);
  }

  public void stop(){
    feederMotor.set(0);
  }
}
