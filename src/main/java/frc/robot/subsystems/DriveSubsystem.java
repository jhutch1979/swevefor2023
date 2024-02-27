// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private int periodicTimer = 1;
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
        "frontLeft",
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftDriveMotorReversed,
          DriveConstants.kFrontLeftTurningMotorReversed,
          DriveConstants.kFrontLeftAnalogEncoderOffset);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
        "rearLeft",
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPort,
          DriveConstants.kRearLeftDriveMotorReversed,
          DriveConstants.kRearLeftTurningMotorReversed,
          DriveConstants.kRearLeftAnalogEncoderOffset);

  private final SwerveModule m_frontRight =
      new SwerveModule(
        "frontRight",
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPort,
          DriveConstants.kFrontRightDriveMotorReversed,
          DriveConstants.kFrontRightTurningMotorReversed,
          DriveConstants.kFrontRightAnalogEncoderOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
        "rearRight",
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPort,
          DriveConstants.kRearRightDriveMotorReversed,
          DriveConstants.kRearRightTurningMotorReversed,
          DriveConstants.kRearRightAnalogEncoderOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private boolean feildOrientation = true;
  private PIDController rotPid = new PIDController(.03, 0, 0);
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("limelight");
  private NetworkTable shooterTable = inst.getTable("limelight-shooter");
  private DoubleSubscriber HorizontalOfsetSub;
  private DoubleSubscriber ShooterHorizontalOfsetSub;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });
          
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    new Thread(()-> {
      try{
        Thread.sleep(1000);
        zeroHeading();
        m_gyro.setAngleAdjustment(0);
      }catch(Exception e){

      }
    }).start();
    HorizontalOfsetSub = table.getDoubleTopic("tx").subscribe(0.0);
    ShooterHorizontalOfsetSub = shooterTable.getDoubleTopic("tx").subscribe(0.0);
    rotPid.setTolerance(1);
    rotPid.setIntegratorRange(-0.25,0.25);
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getRobotRelativeSpeed, 
      this::driveRobotRelative, 
      Constants.DriveConstants.pathconfig, 
      () -> { return false; }, 
      this);
      //resetPose(new Pose2d(3, 7, new Rotation2d()));
  }

  @Override
  public void periodic() {
    if (periodicTimer > 10){
    m_frontLeft.update();
    m_frontRight.update();
    m_rearLeft.update();
    m_rearRight.update();
    SmartDashboard.putNumber("gyro heading", m_gyro.getAngle());
    SmartDashboard.putNumber("pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("roll", m_gyro.getRoll());
    SmartDashboard.putNumber("distance", getAverageDistance());
    SmartDashboard.putString("pose", getPose().toString());
    
    periodicTimer = 0;
    }
  
    periodicTimer++;
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose){
    resetOdometry(pose);
  }
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  public void switchDrive(){
    feildOrientation = !feildOrientation;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double scale, double xSpeed, double ySpeed, double rot) {
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("y speed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.maxAngularVelocity;
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            feildOrientation
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveWhilePickup(double scale, double xSpeed){
    SmartDashboard.putNumber("tx", HorizontalOfsetSub.get());
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeed = 0;
  
    double rot = rotPid.calculate(HorizontalOfsetSub.get(),0);
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveWhileShhoting(double scale, double xSpeed, double ySpeed, double rot){
    if (ShooterHorizontalOfsetSub.get() == 0){
      drive(scale, xSpeed, ySpeed, rot);
      return;
    }
    SmartDashboard.putNumber("tx", ShooterHorizontalOfsetSub.get());
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
  
    rot = rotPid.calculate(ShooterHorizontalOfsetSub.get(),0);
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            feildOrientation
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelativeSim(ChassisSpeeds speeds) {
    var swerveModuleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("flSwerveState", swerveModuleStates[0].angle.getDegrees());
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    var swerveModuleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SmartDashboard.putString("speed x", speeds.toString());
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
    m_gyro.setAngleAdjustment(0);
    m_gyro.reset();
    
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
   // 
  }

  public void stop(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void switchBrake(){
    m_frontLeft.switchBrake();
    m_frontRight.switchBrake();
    m_rearLeft.switchBrake();
    m_rearRight.switchBrake();
  }

  public double getRoll(){
    return m_gyro.getRoll();
  }


  
  public double getPitch(){
    return m_gyro.getPitch();
  }

  public double getAverageDistance(){
    double distance = 
      (Math.abs(m_frontLeft.getDistance()) +
      Math.abs(m_frontRight.getDistance()) +
      Math.abs(m_rearLeft.getDistance())+
      Math.abs(m_rearRight.getDistance()))/4;
      return distance;
  }

  public void setX() {
    m_frontLeft.setAngleForX(45);
    m_frontRight.setAngleForX(-45);
    m_rearLeft.setAngleForX(-45);
    m_rearRight.setAngleForX(45);
  }

  public ChassisSpeeds getRobotRelativeSpeed(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }

}
