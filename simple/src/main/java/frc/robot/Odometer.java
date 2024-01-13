package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;

public class Odometer {
    private static final double kGearRatio = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    private static final double kCountsPerMotorShaftRev = 12.0;
    private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
    private static final double kWheelDiameterInch = 2.3622; // 60 mm

    private final XRPGyro m_gyro = new XRPGyro();

    // The XRP has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);

    private final long m_updateMillis;

    private double m_x;
    private double m_y;
    private double m_inchesLeftPrevious;
    private double m_inchesRightPrevious;
    private long m_tPrevious;

    public Odometer(long updateMillis) {
        m_updateMillis = updateMillis;
        // Use inches as unit for encoder distances
        m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
        m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    }

    public void tare() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();

        m_x = 0;
        m_y = 0;
        m_inchesLeftPrevious = m_leftEncoder.getDistance();
        m_inchesRightPrevious = m_rightEncoder.getDistance();
        m_tPrevious = System.currentTimeMillis();
    }

    public void update(boolean putOnSmartDashboard) {
        long t = System.currentTimeMillis();
        double heading = heading();

        if (t - m_tPrevious > m_updateMillis) {
            double inchesLeft = m_leftEncoder.getDistance();
            double inchesRight = m_rightEncoder.getDistance();

            double deltaInchesLeft = inchesLeft - m_inchesLeftPrevious;
            double deltaInchesRight = inchesRight - m_inchesRightPrevious;
            double deltaInches = (deltaInchesLeft + deltaInchesRight) / 2.0;

            m_x += deltaInches * -Math.sin(Math.toRadians(heading));
            m_y += deltaInches * Math.cos(Math.toRadians(heading));

            m_inchesLeftPrevious = inchesLeft;
            m_inchesRightPrevious = inchesRight;
            m_tPrevious = t;
        }

        if (putOnSmartDashboard) {
            SmartDashboard.putNumber("Odometer Heading", heading);
            SmartDashboard.putNumber("Odometer X", m_x);
            SmartDashboard.putNumber("Odometer Y", m_y);
        }
    }

    public double x() {
        return m_x;
    }

    public double y() {
        return m_y;
    }

    public double heading() {
        return 360 - m_gyro.getAngleZ();
    }
}
