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
    SmartDashboard.putNumber("Gyro Heading", 360 - m_gyro.getAngleZ());
    SmartDashboard.putNumber("Ultrasonic Range", m_ultrasonicSensor.getVoltage());
    SmartDashboard.putBoolean("Onboard User Button", m_onboardUserButton.get());

    m_lineSensor.update(true);
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    m_drivetrain.resetEncoders();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double diff = m_lineSensor.getDifference(SignalProcessingType.NONE);

    // MARK: Example - Bang-bang controller
    if (diff > 0.25) {
      m_drivetrain.arcadeDrive(0, -0.32);
    } else if (diff < -0.25) {
      m_drivetrain.arcadeDrive(0, +0.32);
    } else {
      m_drivetrain.arcadeDrive(0.35, 0);
    }

    // MARK: Example - P controller
    // double error = 0 - diff;
    // // Compute output turn strength using a P controller.
    // // Usually `turnSpeed` would just be `p * error`, but the
    // // motors won't actually move until ~25% power, so we
    // // add that on as a baseline
    // double p = 0.12;
    // double turnSpeed = Math.signum(error) * 0.25 + p * error;
    // // Send command to motors
    // m_drivetrain.arcadeDrive(0.4, turnSpeed);
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
