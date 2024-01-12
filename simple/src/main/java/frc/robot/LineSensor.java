package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineSensor {
    public enum SignalProcessingType {
        NONE,
        MEAN,
        MEDIAN,
        EMA,
    }

    // Sensors
    private final AnalogInput m_sensorLeft = new AnalogInput(0);
    private final AnalogInput m_sensorRight = new AnalogInput(1);

    // Exponential Moving Averages for the reflectance values
    private EMA m_emaLeft;
    private EMA m_emaRight;
    private EMA m_emaDiff;

    // Sliding window data collection for the reflectance values
    private SlidingWindow m_slidingWindowLeft;
    private SlidingWindow m_slidingWindowRight;
    private SlidingWindow m_slidingWindowDiff;

    public LineSensor(double emaGain, int slidingWindowLength) {
        m_emaLeft = new EMA(emaGain);
        m_emaRight = new EMA(emaGain);
        m_emaDiff = new EMA(emaGain);

        m_slidingWindowLeft = new SlidingWindow(slidingWindowLength);
        m_slidingWindowRight = new SlidingWindow(slidingWindowLength);
        m_slidingWindowDiff = new SlidingWindow(slidingWindowLength);
    }

    public void init() {
        double reflectanceLeft = m_sensorLeft.getVoltage();
        double reflectanceRight = m_sensorRight.getVoltage();

        // Initialize EMAs
        m_emaLeft.set(reflectanceLeft);
        m_emaRight.set(reflectanceRight);
        m_emaDiff.set(reflectanceLeft - reflectanceRight);

        // Initialize sliding windows
        m_slidingWindowLeft.fill(reflectanceLeft);
        m_slidingWindowRight.fill(reflectanceRight);
        m_slidingWindowDiff.fill(reflectanceLeft - reflectanceRight);
    }

    public void update(boolean putOnSmartDashboard) {
        double reflectanceLeft = m_sensorLeft.getVoltage();
        double reflectanceRight = m_sensorRight.getVoltage();

        // Update EMAs
        m_emaLeft.update(reflectanceLeft);
        m_emaRight.update(reflectanceRight);
        m_emaDiff.update(reflectanceLeft - reflectanceRight);

        // Update sliding windows
        m_slidingWindowLeft.update(reflectanceLeft);
        m_slidingWindowRight.update(reflectanceRight);
        m_slidingWindowDiff.update(reflectanceLeft - reflectanceRight);

        if (putOnSmartDashboard) {
            SmartDashboard.putNumber("Reflectance (L) Raw", reflectanceLeft);
            SmartDashboard.putNumber("Reflectance (L) EMA", m_emaLeft.get());
            SmartDashboard.putNumber("Reflectance (L) Mean", m_slidingWindowLeft.mean());
            SmartDashboard.putNumber("Reflectance (L) Median", m_slidingWindowLeft.median());

            SmartDashboard.putNumber("Reflectance (R) Raw", reflectanceLeft);
            SmartDashboard.putNumber("Reflectance (R) EMA", m_emaRight.get());
            SmartDashboard.putNumber("Reflectance (R) Mean", m_slidingWindowRight.mean());
            SmartDashboard.putNumber("Reflectance (R) Median", m_slidingWindowRight.median());

            SmartDashboard.putNumber("Reflectance (Δ) Raw", reflectanceRight - reflectanceLeft);
            SmartDashboard.putNumber("Reflectance (Δ) EMA", m_emaDiff.get());
            SmartDashboard.putNumber("Reflectance (Δ) Mean", m_slidingWindowDiff.mean());
            SmartDashboard.putNumber("Reflectance (Δ) Median", m_slidingWindowDiff.median());
        }        
    }

    public double getLeft(SignalProcessingType signalProcessingType) {
        return get(m_emaLeft, m_slidingWindowLeft, signalProcessingType);
    }

    public double getRight(SignalProcessingType signalProcessingType) {
        return get(m_emaRight, m_slidingWindowRight, signalProcessingType);
    }

    public double getDifference(SignalProcessingType signalProcessingType) {
        return get(m_emaDiff, m_slidingWindowDiff, signalProcessingType);
    }

    private double get(EMA ema, SlidingWindow slidingWindow, SignalProcessingType signalProcessingType) {
        switch (signalProcessingType) {
            case NONE:
                return slidingWindow.latest();
            case MEAN:
                return slidingWindow.mean();
            case EMA:
                return ema.get();
            case MEDIAN:
            default:
                return slidingWindow.median();
        }
    }
}
