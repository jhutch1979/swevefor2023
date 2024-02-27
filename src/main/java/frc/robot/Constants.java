// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Swerve {

    public static final double angleGearRatio = (150.0 / 7.0);
    public static final boolean angleInvert = true;
    public static final double voltageComp = 12.0;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second  //4.5
    public static final double maxAngularVelocity = 6; // 11.5

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;

    public static final double angleConversionFactor = 360.0 / angleGearRatio;
  }
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kRearLeftTurningMotorPort = 13;
    public static final int kFrontRightTurningMotorPort = 12;
    public static final int kRearRightTurningMotorPort = 14;

    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kFrontRightTurningEncoderPort = 1;
    public static final int kRearRightTurningEncoderPort = 3;
    public static final int kFrontLeftTurningEncoderPort = 0;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kRearLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kRearRightTurningMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    //public static final double kFrontLeftAnalogEncoderOffset = 4.0455;
    //public static final double kRearLeftAnalogEncoderOffset = 2.8235;
    //public static final double kFrontRightAnalogEncoderOffset = 3.7955;
    //public static final double kRearRightAnalogEncoderOffset = .640;

    public static final double kFrontLeftAnalogEncoderOffset = 49.87;
    public static final double kRearLeftAnalogEncoderOffset = 341.1;
    public static final double kFrontRightAnalogEncoderOffset = 34.64;
    public static final double kRearRightAnalogEncoderOffset = 214.67;

    // Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = 0.61;
    
    // Distance between front and back wheels on robot in meters
    public static final double kWheelBase = 0.61;

    public static final Translation2d flModuleOffset = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d frModuleOffset = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d blModuleOffset = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d brModuleOffset = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            flModuleOffset,
            frModuleOffset,
            blModuleOffset,
            brModuleOffset);

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double maxAngularVelocity = 10;

    public static HolonomicPathFollowerConfig pathconfig = new HolonomicPathFollowerConfig( 
                    new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                    1,//kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                    flModuleOffset.getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            );
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedDegreesPerSecond = 540;
    public static final double kMaxModuleAngularAccelerationDegreesPerSecondSquared = 720;

    public static final double kEncoderCPR = 1.085;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveGearRatio = 1/6.75;
    public static final double kDriveEncoderDistancePerPulse = 
        // Assumes the encoders are directly mounted on the wheel shafts
       (kWheelDiameterMeters * Math.PI) / kEncoderCPR* kDriveGearRatio;

    private static final double kturningEncoderCountsPerRevolution = 1;
    private static final double kTurningEncoderGearRatio = .04667;
    public static final double kTurningEncoderDistancePerPulse =
        kturningEncoderCountsPerRevolution * kTurningEncoderGearRatio * (2 * Math.PI);

    public static final double kPModuleTurningController = .5;

    public static final double kPModuleDriveController = .75;
    public static double kPModuleDriveSlewRate = 2;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static final class ElevatorConstants{

    public static final int potentiometerAnologID = 6;
    public static final int MotorID = 3;
    public static final double Speed = .8;

  }

  public static final class ExtenderConstants{

    public static final int absoluteEncoderPort = 0;
    public static final int[] encoderPorts = {1,2};
    public static final int MotorID = 2;
    public static final double Speed = .3;

  }

  public static final class IntakeConstants{
    public static final int motorID = 1;
    public static int motor2ID =4;

  }
  public static final class AutonConfig{
    public static final TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    
  }
  public static final class ShooterAnglePid{
    public static double kP = 0.0005; 
    public static double kI = .00000025;
    public static double kD = 0; 
    public static double kIz = 0; 
    public static double kFF = 0; 
    public static double kMaxOutput = 1; 
    public static double kMinOutput = -1;
  }
}
