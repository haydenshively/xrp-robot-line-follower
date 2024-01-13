package frc.robot.signalprocessing;

public class EMA {
    private final double m_gain;

    private double m_value;

    public EMA(double gain) {
        if (gain < 0 || gain > 1) {
            throw new Error("ExponentialMovingAverage gain must be between 0 and 1");
        }
        m_gain = gain;
    }

    public void set(double value) {
        m_value = value;
    }

    public double get() {
        return m_value;
    }

    public double update(double value) {
        m_value = m_gain * m_value + (1.0 - m_gain) * value;
        return m_value;
    }
}
