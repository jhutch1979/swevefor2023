// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberReverseCommand;
import frc.robot.commands.ClimberRunCommand;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ShooterStopCommand;
import frc.robot.commands.climberPreSetCommand;
import frc.robot.commands.climberdeployCommand;
import frc.robot.commands.feederRunCommand;
import frc.robot.commands.intakeRunInCommand;
import frc.robot.commands.intakeRunOutCommand;
import frc.robot.commands.intakeStopCommand;
import frc.robot.commands.AutoIntakeAndDrivePickup;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.shooterRunCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // paths
  
  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  //XboxController m_manipulator = new XboxController(1);
  //Joystick m_driverStation = new Joystick(2);
  // The robot's subsystems
  private static DigitalInput noteSensor = new DigitalInput(0); // Limit switch on DIO 0
  Trigger noteSensorTrigger = new Trigger(noteSensor::get);
  
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(noteSensor);
  public final ClimberSubsystem m_climberSubsytem = new ClimberSubsystem();
  
  //public AddressableLED m_led;
  //public AddressableLEDBuffer m_ledBuffer;
  //public boolean LightsYellow = false;

  private static final String kDefaultAuto = "Default";
  private static final String kSlowBalance = "Slow";
  
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  


  

  //ProfiledPIDController thetaController =
  //      new ProfiledPIDController(
  //          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


      NamedCommands.registerCommand("Intake", new AutoIntakeAndDrivePickup(m_IntakeSubsystem, m_robotDrive, noteSensor));
      NamedCommands.registerCommand("ShootAuto", new AutoShoot(m_ShooterSubsystem, m_FeederSubsystem).withTimeout(1.5));
      
   

   //m_led.setData(m_ledBuffer);
    // Configure the button bindings
    configureButtonBindings();
    //CameraServer.startAutomaticCapture();
   // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new DriveWithJoystick(m_driverController, m_robotDrive));
    //m_elevator.setDefaultCommand(new ElevatorStop(m_elevator));
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Balance", kCustomAuto);
    m_chooser.addOption("Slow Balance", kSlowBalance);
    SmartDashboard.putData("Auto choices", m_chooser);

  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 6).whileTrue(
        new intakeRunOutCommand(m_IntakeSubsystem)
    );
    new JoystickButton(m_driverController, 7).onTrue(
        new InstantCommand(
            () ->
                m_robotDrive.switchDrive())
    );
    new JoystickButton(m_driverController, 11).onTrue(
      new InstantCommand(
        () ->
        m_robotDrive.switchBrake()
      )
    );
    //new JoystickButton(m_driverStation, 7).onTrue(
    //  new InstantCommand(
    //    () ->
    //    m_robotDrive.switchBrake()
    //  )
    //);
    new JoystickButton(m_driverController, 1).whileTrue(
      new shooterRunCommand(m_ShooterSubsystem)
    );
    new JoystickButton(m_driverController, 1).onFalse(
      new ShooterStopCommand(m_ShooterSubsystem)
    );

    new JoystickButton(m_driverController, 5).whileTrue(
      new feederRunCommand(m_FeederSubsystem)
    );

    new JoystickButton(m_driverController, 2).whileTrue(
      new intakeRunInCommand(m_IntakeSubsystem, noteSensor)
    );
    noteSensorTrigger.onFalse(
      new intakeStopCommand(m_IntakeSubsystem)
    );

    new JoystickButton(m_driverController, 8).whileTrue(
      new ClimberRunCommand(m_climberSubsytem)
      );

    new JoystickButton(m_driverController, 9).whileTrue(
      new climberPreSetCommand(m_ShooterSubsystem)
      );

    new JoystickButton(m_driverController, 10).whileTrue(
      new climberdeployCommand(m_ShooterSubsystem)
      );
    new JoystickButton(m_driverController, 12).whileTrue(
      new ClimberReverseCommand(m_climberSubsytem)
    );

      

    
    //new JoystickButton(m_driverController, 8).onTrue(
    //  new park2part(m_robotDrive)
    //);
    
    
    
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //if(m_driverStation.getRawButton(1)){
    //  return null;
    //} else if(m_driverStation.getRawButton(2)) {
    //  return null;
    //} else {

      //return new AutoIntakeAndDrivePickup(m_IntakeSubsystem, m_robotDrive, noteSensor);
     return new PathPlannerAuto("Auto");
    //}

    /*
    m_robotDrive.switchBrake();
    //return new ParkOnChargeStationCommand(m_robotDrive, thetaController);
    // Create config for trajectory
     
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory shortTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3.5, 0, new Rotation2d(0)),
                config);
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(2,0) ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(5, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand;
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    m_autoSelected = m_chooser.getSelected();
    switch (m_autoSelected) {
      case kSlowBalance:
        return new park2part(m_robotDrive);
      case kCustomAuto:
        // Put custom auto code here
        swerveControllerCommand =
        new SwerveControllerCommand(
            shortTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        break;
    }
    //SwerveControllerCommand swerveControllerCommand =
    //    new SwerveControllerCommand(
    //        exampleTrajectory,
    //        m_robotDrive::getPose, // Functional interface to feed supplier
    //        DriveConstants.kDriveKinematics,
//
    //        // Position controllers
    //        new PIDController(AutoConstants.kPXController, 0, 0),
    //        new PIDController(AutoConstants.kPYController, 0, 0),
    //        thetaController,
    //        m_robotDrive::setModuleStates,
    //        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(1,0, 0, 0));
    */
  }
}
