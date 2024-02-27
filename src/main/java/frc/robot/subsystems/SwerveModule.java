// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  

  private final String m_name;
  private final CANSparkFlex m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private boolean isBrake = false;

  private Rotation2d lastAngle;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final SlewRateLimiter filter = new SlewRateLimiter(ModuleConstants.kPModuleDriveSlewRate);
  private AnalogInput absoluteEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  private double AnalogEncoderOffset;

  private final SparkPIDController angleController;
  

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedDegreesPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationDegreesPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param name                   the name of the module
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param turningEncoderChannel  The channels of the turning encoder.
   * @param driveMotorReversed   Whether the drive encoder is reversed.
   * @param turningMotorReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double encoderOffset) {
        AnalogEncoderOffset = encoderOffset;
        absoluteEncoder = new AnalogInput(turningEncoderChannel);
        m_name = name;
        m_driveMotor = new CANSparkFlex(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();

        m_turningEncoder = m_turningMotor.getEncoder();

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse/60);
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
        // m_driveEncoder.setReverseDirection(driveEncoderReversed);
        m_driveMotor.setInverted(driveMotorReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning
        // encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
        //m_turningEncoder.setDistancePerRotation(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        // m_turningEncoder.setReverseDirection(turningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderDistancePerPulse);
        m_turningMotor.setInverted(turningMotorReversed);
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        angleController = m_turningMotor.getPIDController();



        configAngleMotor();
        lastAngle = getState().angle;
  }

  private Rotation2d getAngle(){
    return Rotation2d.fromDegrees(m_turningEncoder.getPosition());
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), getAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(Units.degreesToRadians(getAbsoluteEncoder())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    //stops turning back to zero when you release the joystick
    if(desiredState.speedMetersPerSecond < .1){
      stop();
      return;
    }
    // Optimize the reference state to avoid spinning further than 90 degrees
    SmartDashboard.putNumber(m_name + " angle pre optimize", desiredState.angle.getDegrees());
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);//new Rotation2d(m_turningEncoder.getPosition()));
    SmartDashboard.putNumber(m_name + " angle post optimize", state.angle.getDegrees());
    // Calculate the drive output from the drive PID controller.
    //final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
    final double driveOutput = state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
    // Calculate the turning motor output from the turning PID controller.
    double turnOutput;
    //if(Math.abs((m_turningEncoder.getPosition()) - state.angle.getDegrees())< 2 || Math.abs(driveOutput) < .25){
    //  turnOutput =0;
    //}else{
    turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(),
       state.angle.getRadians());
    //}
    // Calculate the turning motor output from the turning PID controller.
    //SmartDashboard.putNumber(m_name + "driveSpeed", driveOutput);
    // SmartDashboard.putNumber(m_name +"turnSpeed", turnOutput);
    //SmartDashboard.putNumber(m_name + "des mps", desiredState.speedMetersPerSecond);
    //SmartDashboard.putNumber(m_name + "cur MPS", m_driveEncoder.getVelocity());
    //SmartDashboard.putNumber(m_name +" drive out", driveOutput);
    //SmartDashboard.putNumber(m_name +" current Angle",m_turningEncoder.getPosition());
    //SmartDashboard.putNumber(m_name + " desired angle", state.angle.getRadians());
   
    //SmartDashboard.putNumber(m_name + " turning error", m_turningEncoder.getPosition() - state.angle.getRadians());
    m_driveMotor.set(filter.calculate(driveOutput));
    //SmartDashboard.putNumber(m_name + " turn output", turnOutput);
    //m_turningMotor.set(turnOutput);
    setAngle(state);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(getAbsoluteEncoder());
  }

  public CANSparkFlex driveFlex() {
    return m_driveMotor;
  }

  public CANSparkMax turnSpark() {
    return m_turningMotor;
  }

  public void stop(){
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  public double getAbsoluteEncoder(){
    double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
    angle *= 360;
    angle -= AnalogEncoderOffset;
    return angle ;
  }

  public double getRawAbsoluteEncoder(){
    return (absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V())* 360;
  }

  public void update(){
    //SmartDashboard.putNumber(m_name + "lastAngle", lastAngle.getDegrees());
    //SmartDashboard.putNumber(m_name + "wheel angle", getAngle().getDegrees());
    SmartDashboard.putBoolean("brake mode", isBrake);
    //SmartDashboard.putNumber(m_name + "Raw Angle", getRawAbsoluteEncoder());
    SmartDashboard.putNumber(m_name + " distance", m_driveEncoder.getPosition());
  }

  public void switchBrake(){
    if(isBrake){
      m_driveMotor.setIdleMode(IdleMode.kCoast);
      isBrake = false;
      return;
    }
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    isBrake = true;
  }
  public double getDistance(){
    return m_driveEncoder.getPosition();
  }

  private void configAngleMotor() {
    m_turningMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_turningMotor, Usage.kPositionOnly);
    m_turningMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    m_turningMotor.setInverted(Constants.Swerve.angleInvert);
    m_turningMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    m_turningEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    m_turningMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    m_turningMotor.burnFlash();
    Timer.delay(1);
    resetEncoders();
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
    //SmartDashboard.putNumber(m_name + "desired angle", angle.getDegrees());
    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void setAngleForX(double angle) {
    m_driveMotor.set(0);
    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = new Rotation2d(Units.degreesToRadians(angle));
  }


}
