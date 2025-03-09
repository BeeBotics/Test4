// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Arm;
import frc.robot.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private elevator elevator;
  private Arm arm;

  private Constants constants;
  private XboxController m_Controller;
  private XboxController m_DriveController;
  private SparkMax climbMotor;
  private SparkMax algaeMotor;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    elevator = new elevator();
    arm = new Arm();
    constants = new Constants();

    climbMotor = new SparkMax(20, MotorType.kBrushless);
    algaeMotor = new SparkMax(12, MotorType.kBrushless);

    m_Controller = new XboxController(1);
    m_DriveController = new XboxController(0);

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
       CommandScheduler.getInstance().run();

    DogLog.log("Debug/SwerveState", new SwerveModuleState());
    DogLog.log(
        "Debug/SwerveStates",
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        });
    DogLog.log("Debug/Position", elevator.getHeight());
    DogLog.log("Debug/Json", "{\"test\": \"json\"}", "json");
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {DogLog.decreaseFault("ExampleFault");}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     SmartDashboard.putNumber("Position", elevator.getHeight());
     SmartDashboard.putNumber("Rotation", arm.getRotation());

    if (m_Controller.getXButton() == true) {     // L4 
      elevator.setPosition(-0.95); 
      arm.setRotation(-0.021);
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (m_Controller.getYButton() == true) {     // L3 
      elevator.setPosition(-0.1); 
      arm.setRotation(-0.021);
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (m_Controller.getBButton() == true) {     // L2 
      elevator.setPosition(0); 
      arm.setRotation(-0.023);
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (m_Controller.getAButton() == true) {     // Yoink
  elevator.setPosition(0.5); 
 
  arm.setRotation(0);
}
if (m_Controller.getRightBumperButton() == true) {     // Reset
  arm.setRotation(0.01);
  arm.resetRotation();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (m_Controller.getXButton() == false && m_Controller.getYButton() == false && m_Controller.getAButton() == false && m_Controller.getBButton() == false && m_Controller.getRightBumperButton() == false) {
  arm.setRotation(0);
  elevator.setPosition(0); 
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

//if (m_Controller.getRightBumperButton() == true) {
   //   algaeMotor.set(-0.3);
   // }
  //  else if (m_Controller.getRightBumperButtonReleased() == true) {
  //    algaeMotor.set(0);
//}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

if (m_Controller.getLeftBumperButton() == true) {
  algaeMotor.set(0.5);
}
else if (m_Controller.getLeftBumperButtonReleased() == true) {
  algaeMotor.set(0);
}

    climbMotor.set(-m_Controller.getLeftY()/2);

}


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Position", elevator.getHeight());
    SmartDashboard.putNumber("Rotation", arm.getRotation());
  }
}
