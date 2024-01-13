package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineSensor extends XRPReflectanceSensors {
    public enum MistakeType {
        TOO_FAR_LEFT,
        TOO_FAR_RIGHT,
    }

    private final double m_mistakeThreshold;

    private final double m_recoveryThreshold;

    private final SignalProcessingType m_signalProcessingType;

    private MistakeType m_mostRecentMistake = null;

    public LineSensor(double mistakeThreshold, double correctiveOvershootThreshold,
            SignalProcessingType signalProcessingType, double emaGain, int slidingWindowLength) {
        super(emaGain, slidingWindowLength);

        m_mistakeThreshold = mistakeThreshold;
        m_recoveryThreshold = correctiveOvershootThreshold;
        m_signalProcessingType = signalProcessingType;
    }

    public void update(boolean putOnSmartDashboard) {
        super.update(putOnSmartDashboard);

        double reflectance = super.getDifference(m_signalProcessingType);

        // If reflectance is greater than the mistake threshold (in either direction),
        // set `m_mostRecentMistake`
        if (reflectance > m_mistakeThreshold) {
            m_mostRecentMistake = MistakeType.TOO_FAR_RIGHT;
        } else if (reflectance < -m_mistakeThreshold) {
            m_mostRecentMistake = MistakeType.TOO_FAR_LEFT;
        }
        // There are two ways for reflectance to return to ~0:
        // (1) actually get back on track
        // (2) continue off-roading until *both* sensors are over white
        // To tell the difference between those two, we check for overshoot in the opposite direction.
        // If we're taking corrective action, it's likely that we'll go too far and see `reflectance`
        // swing the other way. So we use that as an indicator for clearing the `m_mostRecentMistake`
        // state.
        if ((m_mostRecentMistake == MistakeType.TOO_FAR_LEFT && reflectance > m_recoveryThreshold)
                || (m_mostRecentMistake == MistakeType.TOO_FAR_RIGHT && reflectance < -m_recoveryThreshold)) {
            m_mostRecentMistake = null;
        }

        if (putOnSmartDashboard) {
            SmartDashboard.putString("Mistake", m_mostRecentMistake == null ? "NONE" : m_mostRecentMistake.toString());
        }
    }

    public MistakeType mostRecentMistake() {
        return m_mostRecentMistake;
    }
}
