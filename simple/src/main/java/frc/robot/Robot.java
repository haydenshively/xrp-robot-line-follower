// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPServo;

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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // NOTE: need latest
  // [firmware](https://github.com/wpilibsuite/xrp-wpilib-firmware/releases/tag/v1.0.1)
  // and latest
  // [VSCode](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.1.1-beta-4)
  // Other docs:
  // https://docs.wpilib.org/en/latest/docs/xrp-robot/getting-to-know-xrp.html

  private final XboxController m_controller = new XboxController(0);

  private final XRPDrivetrain m_drivetrain = new XRPDrivetrain();

  private final XRPServo m_servoArm = new XRPServo(4);

  private final AnalogInput m_reflectanceSensorL = new AnalogInput(0);

  private final AnalogInput m_reflectanceSensorR = new AnalogInput(1);

  private final AnalogInput m_ultrasonic = new AnalogInput(2);

  private final DigitalInput m_onboardUserButton = new DigitalInput(0);

  private final DigitalOutput m_onboardGreenLED = new DigitalOutput(1);

  // Exponential Moving Averages for the reflectance values
  private EMA leftReflectanceEMA = new EMA(0.9);
  private EMA rightReflectanceEMA = new EMA(0.9);
  private EMA diffReflectanceEMA = new EMA(0.9);

  // Sliding window data collection for the reflectance values
  private SlidingWindow leftReflectanceSlidingWindow = new SlidingWindow(15);
  private SlidingWindow rightReflectanceSlidingWindow = new SlidingWindow(15);
  private SlidingWindow diffReflectanceSlidingWindow = new SlidingWindow(15);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    double reflectanceL = m_reflectanceSensorL.getVoltage();
    double reflectanceR = m_reflectanceSensorR.getVoltage();

    // Initialize EMAs
    leftReflectanceEMA.set(reflectanceL);
    rightReflectanceEMA.set(reflectanceR);
    diffReflectanceEMA.set(reflectanceL - reflectanceR);

    // Initialize sliding windows
    leftReflectanceSlidingWindow.fill(reflectanceL);
    rightReflectanceSlidingWindow.fill(reflectanceR);
    diffReflectanceSlidingWindow.fill(reflectanceL - reflectanceR);
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
    double reflectanceL = m_reflectanceSensorL.getVoltage();
    double reflectanceR = m_reflectanceSensorR.getVoltage();

    // Update EMAs
    leftReflectanceEMA.update(reflectanceL);
    rightReflectanceEMA.update(reflectanceR);
    diffReflectanceEMA.update(reflectanceL - reflectanceR);

    // Update sliding windows
    leftReflectanceSlidingWindow.update(reflectanceL);
    rightReflectanceSlidingWindow.update(reflectanceR);
    diffReflectanceSlidingWindow.update(reflectanceL - reflectanceR);

    // Put data on the SmartDashboard
    SmartDashboard.putNumber("Reflectance (L) Raw", reflectanceL);
    SmartDashboard.putNumber("Reflectance (L) EMA", leftReflectanceEMA.get());
    SmartDashboard.putNumber("Reflectance (L) Mean", leftReflectanceSlidingWindow.mean());
    SmartDashboard.putNumber("Reflectance (L) Median", leftReflectanceSlidingWindow.median());

    SmartDashboard.putNumber("Reflectance (R) Raw", reflectanceL);
    SmartDashboard.putNumber("Reflectance (R) EMA", rightReflectanceEMA.get());
    SmartDashboard.putNumber("Reflectance (R) Mean", rightReflectanceSlidingWindow.mean());
    SmartDashboard.putNumber("Reflectance (R) Median", rightReflectanceSlidingWindow.median());

    SmartDashboard.putNumber("Reflectance (Δ) Raw", reflectanceR - reflectanceL);
    SmartDashboard.putNumber("Reflectance (Δ) EMA", diffReflectanceEMA.get());
    SmartDashboard.putNumber("Reflectance (Δ) Mean", diffReflectanceSlidingWindow.mean());
    SmartDashboard.putNumber("Reflectance (Δ) Median", diffReflectanceSlidingWindow.median());

    SmartDashboard.putNumber("Ultrasonic Range", m_ultrasonic.getVoltage());
    SmartDashboard.putBoolean("Onboard User Button", m_onboardUserButton.get());
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    m_drivetrain.resetEncoders();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Uncomment this if you want to use the Xbox controller:
    // m_drivetrain.arcadeDrive(m_controller.getLeftY(), m_controller.getLeftX());

    // You can (should) move these constants to the top of the file:
    double deadband = 0.2;
    double p = 0.15;

    // When driving straight, the two reflectance sensors should report
    // approximately the same value. If they differ, we know we're veering
    // off course, so that's our "error" -- the difference between our
    // current state and desired state.
    double error = -1 * diffReflectanceSlidingWindow.median();
    // Apply deadband
    if (Math.abs(error) < deadband) {
      error = 0;
    }
    // Compute output turn strength using a P controller
    double output = p * error;

    SmartDashboard.putNumber("output", output);

    // Send command to motors
    m_drivetrain.arcadeDrive(0.3, -output);

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
  }
}
