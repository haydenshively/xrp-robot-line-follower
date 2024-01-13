// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import frc.robot.XRPReflectanceSensors.SignalProcessingType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // NOTE: need latest
  // [firmware](https://github.com/wpilibsuite/xrp-wpilib-firmware/releases/tag/v1.0.1)
  // and latest
  // [VSCode](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.1.1-beta-4)
  // Other docs:
  // https://docs.wpilib.org/en/latest/docs/xrp-robot/getting-to-know-xrp.html

  private final XboxController m_controller = new XboxController(0);

  private final XRPDrivetrain m_drivetrain = new XRPDrivetrain();

  private final XRPGyro m_gyro = new XRPGyro();

  private final XRPServo m_servoArm = new XRPServo(4);

  private final XRPReflectanceSensors m_lineSensor = new XRPReflectanceSensors(0.9, 15);

  private final AnalogInput m_ultrasonicSensor = new AnalogInput(2);

  private final DigitalInput m_onboardUserButton = new DigitalInput(0);

  private final DigitalOutput m_onboardGreenLED = new DigitalOutput(1);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_lineSensor.init();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Ultrasonic Range", m_ultrasonicSensor.getVoltage());
    SmartDashboard.putBoolean("Onboard User Button", m_onboardUserButton.get());

    m_lineSensor.update(true);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_drivetrain.resetEncoders();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // When driving straight, the two reflectance sensors should report
    // approximately the same value. If they differ, we know we're veering
    // off course, so that's our "error" -- the difference between our
    // current state and desired state.
    double error = 0 - m_lineSensor.getDifference(SignalProcessingType.MEDIAN);
    // Compute output turn strength using a P controller
    double p = 0.15;
    double output = p * error;
    // Send command to motors
    m_drivetrain.arcadeDrive(0.3, output);

    SmartDashboard.putNumber("output", output);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_drivetrain.arcadeDrive(m_controller.getLeftY(), m_controller.getLeftX());

    if (m_controller.getAButton()) {
      m_servoArm.setAngle(180);
    } else {
      m_servoArm.setAngle(0);
    }

    m_onboardGreenLED.set(m_controller.getBButton());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_servoArm.setAngle(90 - m_lineSensor.getDifference(SignalProcessingType.NONE) * 20);
  }
}
