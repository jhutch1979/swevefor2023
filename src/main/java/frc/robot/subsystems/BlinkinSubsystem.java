// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BlinkinSubsystem extends SubsystemBase {
  private Spark blinkin = new Spark(0);
  /** Creates a new BlinkinSubsystem. */
  public BlinkinSubsystem() {
    changePattern(-0.87 );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void changePattern(double patternId){
    blinkin.set(patternId);
  }
  public void setToAllianceColor(){
    if (DriverStation.getAlliance().isPresent()){
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        changePattern(Constants.BlinkinColors.redAliance);
      } else {
        changePattern(Constants.BlinkinColors.blueAliance);
      }
    } else {
      changePattern(Constants.BlinkinColors.defaultBlinkinPattern);
    }
  }
}
