// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private static final CANSparkMax climberMotor = new CANSparkMax(24, MotorType.kBrushless);
  private static final DigitalInput isClimberDown = new DigitalInput(1);
  private static RelativeEncoder climberEncoder;
  private static SparkPIDController HeightPid;
  /** Creates a new FeederSubsystem. */
  public ClimberSubsystem() {
    climberMotor.setInverted(true);
    climberMotor.setIdleMode(IdleMode.kBrake);
    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0);
    HeightPid = climberMotor.getPIDController();
    HeightPid.setP(Constants.ClimberHeightPid.kP);
    HeightPid.setI(Constants.ClimberHeightPid.kI);
    HeightPid.setD(Constants.ClimberHeightPid.kD);
    HeightPid.setIZone(Constants.ClimberHeightPid.kIz);
    HeightPid.setFF(Constants.ClimberHeightPid.kFF);
    HeightPid.setOutputRange(Constants.ClimberHeightPid.kMinOutput, Constants.ShooterAnglePid.kMaxOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Encoder", climberEncoder.getPosition());
    // This method will be called once per scheduler run
  }

  public void run(boolean joystick3Down){
    if (isClimberDown.get() ){
      if (climberEncoder.getPosition()> -10 && !joystick3Down){
        climberMotor.set(0 );
        }else {
          climberMotor.set(1);
        }
      
      
    } else  {
      climberMotor.set(0);
    }
    
  }

  public void CLimberToNumber(double Target){
    HeightPid.setReference(Target, CANSparkMax.ControlType.kPosition);
  }

  public void reverse(){
    climberMotor.set(-.6);
  }

  public void stop(){
    climberMotor.set(0);
  }
}
