// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.relation.RoleNotFoundException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
  private static final CANSparkMax upperShooterMotor = new CANSparkMax(20, MotorType.kBrushless);
  private static final CANSparkMax lowerShooterMotor = new CANSparkMax(21, MotorType.kBrushless);
  private static final CANSparkMax shoterAngleMotor = new CANSparkMax(25,MotorType.kBrushless);
  private static SparkPIDController anglePid;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable shooterTable = inst.getTable("limelight-shooter");
  private DoubleArraySubscriber robotPos;
  private double previousDistance = 0;
  private double targetX;
  private double targetY = 5.5;

  

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
    robotPos = shooterTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {0,0});
    upperShooterMotor.setOpenLoopRampRate(.5);
    lowerShooterMotor.setOpenLoopRampRate(.5);
    upperShooterMotor.enableVoltageCompensation(10);
    lowerShooterMotor.enableVoltageCompensation(10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double distance = (Units.metersToInches(ShooterDistance.get()[1]));//-19.53125)/.84375;
    SmartDashboard.putNumber("distance",getDistance());
    SmartDashboard.putNumber("currentAngle", shoterAngleMotor.getEncoder().getPosition());
    
  }
  private double getDistance(){
    double doubleArray[] = {0,0,0};
    double [] robotPosition = robotPos.get(doubleArray);
    double robotX = robotPosition[0];
    double robotY = robotPosition[1];
    double distance = Math.sqrt((Math.pow(Math.abs(targetX-robotX), 2)+(Math.pow(Math.abs(targetY-robotY), 2))));

    return Units.metersToInches(distance);
  }



  public void setAlliance(){
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red){
        targetX = 16.4846;
        return;
      }
      
    }
    targetX = 0;
  }

  public void runWithDistance(){
    double distance = getDistance();//(Units.metersToInches(ShooterDistance.get()[1]));//-19.53125)/.84375;
    if (distance == 0){
      distance = previousDistance;
    } else {
      previousDistance = distance;
    }
    double multiplier = (DriverStation.isTeleop()) ? 6.7 : 6.7;
    double angleSetPoint;
    //double speed = (ShooterDistance.get()[2] > 75) ? Math.max(.65, ((ShooterDistance.get() - 1.91)*.0026)+0.65): .65;
    if (distance < 60){
      angleSetPoint =50;
    } else {
    angleSetPoint = (distance>125) ? distance*(6.574-((distance -125)*.01)): distance*multiplier; 
    angleSetPoint += 45;
    }
    
    double speed = (distance < 60) ? .75  : .75;
    upperShooterMotor.set(speed);
    lowerShooterMotor.set(speed);
    anglePid.setReference(angleSetPoint, CANSparkMax.ControlType.kPosition);
    
  }

  public void runForAmp(){
    upperShooterMotor.set(.25);
    lowerShooterMotor.set(.32);
    shoterAngleMotor.set(0);
  }
  public void runForFullCourt(){
    upperShooterMotor.set(.6);
    lowerShooterMotor.set(.6);
    anglePid.setReference(500, CANSparkMax.ControlType.kPosition);
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
    //SmartDashboard.putNumber("Current Angle", shoterAngleMotor.getEncoder().getPosition());
    anglePid.setReference(angleSetPoint, CANSparkMax.ControlType.kPosition);
  }
  
  public void shooterReset(){
    shoterAngleMotor.getEncoder().setPosition(0);
  }

}
