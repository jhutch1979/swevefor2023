// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static final CANSparkMax upperShooterMotor = new CANSparkMax(20, MotorType.kBrushless);
  private static final CANSparkMax lowerShooterMotor = new CANSparkMax(21, MotorType.kBrushless);
  private static final CANSparkMax shoterAngleMotor = new CANSparkMax(25,MotorType.kBrushless);
  private static SparkPIDController anglePid;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable shooterTable = inst.getTable("limelight-shooter");
  private DoubleArraySubscriber ShooterDistance;
  

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    upperShooterMotor.setIdleMode(IdleMode.kBrake);
    lowerShooterMotor.setIdleMode(IdleMode.kBrake);
    shoterAngleMotor.setIdleMode(IdleMode.kBrake);
    anglePid = shoterAngleMotor.getPIDController();
    anglePid.setP(Constants.ShooterAnglePid.kP);
    anglePid.setI(Constants.ShooterAnglePid.kI);
    anglePid.setD(Constants.ShooterAnglePid.kD);
    anglePid.setIZone(Constants.ShooterAnglePid.kIz);
    anglePid.setFF(Constants.ShooterAnglePid.kFF);
    anglePid.setOutputRange(Constants.ShooterAnglePid.kMinOutput, Constants.ShooterAnglePid.kMaxOutput);
    ShooterDistance = shooterTable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    upperShooterMotor.setOpenLoopRampRate(.5);
    lowerShooterMotor.setOpenLoopRampRate(.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double distance = (Units.metersToInches(ShooterDistance.get()[2])-19.53125)/.84375;
    SmartDashboard.putNumber("distance",distance);
    
  }

  public void runWithDistance(){
    double distance = (Units.metersToInches(ShooterDistance.get()[2])-19.53125)/.84375;
    double angleSetPoint;
    //double speed = (ShooterDistance.get()[2] > 75) ? Math.max(.65, ((ShooterDistance.get() - 1.91)*.0026)+0.65): .65;
    if (distance < 60){
      angleSetPoint =50;
    } else {
    angleSetPoint = (distance>125) ? distance*(6.574-((distance -125)*.01)): distance*6.7; 
    }
    double speed = (distance < 60) ? .85 : .85;
    upperShooterMotor.set(speed);
    lowerShooterMotor.set(speed);
    anglePid.setReference(angleSetPoint, CANSparkMax.ControlType.kPosition);
  }

  public void stop(){
    upperShooterMotor.set(0);
    lowerShooterMotor.set(0);
    
  }
  public void stopShooter(){
    shoterAngleMotor.set(0);
  }

  public Double getAngle(){
      return shoterAngleMotor.getEncoder().getPosition();
  }

  public void run(){
    upperShooterMotor.set(.85);
    lowerShooterMotor.set(.85);
  }

  public void setAngle(double angleSetPoint){
    SmartDashboard.putNumber("Current Angle", shoterAngleMotor.getEncoder().getPosition());
    anglePid.setReference(angleSetPoint, CANSparkMax.ControlType.kPosition);
  }


}
