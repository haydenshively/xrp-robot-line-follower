package frc.robot;

public class EMA {
    private final double gain;

    private double value;

    public EMA(double gain) {
        if (gain < 0 || gain > 1) {
            throw new Error("ExponentialMovingAverage gain must be between 0 and 1");
        }
        this.gain = gain;
    }

    public void set(double value) {
        this.value = value;
    }

    public double get() {
        return this.value;
    }

    public double update(double value) {
        this.value = this.gain * this.value + (1.0 - this.gain) * value;
        return this.value;
    }
}
