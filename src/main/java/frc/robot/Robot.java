// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  private static NetworkTable table;
  NetworkTableEntry pipelineEntry;
 
  
  
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    table = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    pipelineEntry = table.getEntry("pipeline");
    m_robotContainer.m_blinkin.changePattern(Constants.BlinkinColors.defaultBlinkinPattern);
   
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_pdh.setSwitchableChannel(true);
    m_robotContainer.shooterSetAlliance();
    m_robotContainer.m_blinkin.changePattern(Constants.BlinkinColors.defaultBlinkinPattern);
   
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (isRedAliance()){
      m_robotContainer.m_blinkin.changePattern(Constants.BlinkinColors.redAliance);
      pipelineEntry.setNumber(0);
    } else {
      m_robotContainer.m_blinkin.changePattern(Constants.BlinkinColors.blueAliance);
      pipelineEntry.setNumber(1);
    }
    m_robotContainer.shooterSetAlliance();
    m_pdh.setSwitchableChannel(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (isRedAliance()){
      m_robotContainer.m_blinkin.changePattern(-0.31);
      pipelineEntry.setNumber(0);
    } else {
      m_robotContainer.m_blinkin.changePattern(-0.29);
      pipelineEntry.setNumber(1);
    }
    m_robotContainer.shooterSetAlliance();
    m_pdh.setSwitchableChannel(false);
    m_robotContainer.m_ShooterSubsystem.stop();
    m_robotContainer.m_ShooterSubsystem.stopShooter();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.\
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //if(DriverStation.getAlliance() == Alliance.Red){
    //  for (var i = 0; i < m_robotContainer.m_ledBuffer.getLength(); i++) {
    //    m_robotContainer.m_ledBuffer.setRGB(i, 255, 0, 0);
    //  }
    //  for (var i = 0; i < m_robotContainer.m_ledBuffer2.getLength(); i++) {
    //    m_robotContainer.m_ledBuffer2.setRGB(i, 255, 0, 0);
    //    }
    //} else { 
    //  for (var i = 0; i < m_robotContainer.m_ledBuffer.getLength(); i++) {
    //    m_robotContainer.m_ledBuffer.setRGB(i, 0, 0, 255);
    //  }
    //  for (var i = 0; i < m_robotContainer.m_ledBuffer2.getLength(); i++) {
    //    m_robotContainer.m_ledBuffer2.setRGB(i, 0, 0, 255);
    //    }
    //  }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    //REVPhysicsSim.getInstance().addSparkMax(RobotContainer.m_robotDrive.flDrive, DCMotor.getNEO(1));
    //REVPhysicsSim.getInstance().addSparkMax(RobotContainer.m_robotDrive.frDrive, DCMotor.getNEO(1));
    //REVPhysicsSim.getInstance().addSparkMax(RobotContainer.m_robotDrive.rlDrive, DCMotor.getNEO(1));
    //REVPhysicsSim.getInstance().addSparkMax(RobotContainer.m_robotDrive.rrDrive, DCMotor.getNEO(1));
      
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
      
  }

  public boolean isRedAliance(){
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()){
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
  
}
