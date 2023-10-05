package frc.lib.control.pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class NKSolenoid extends Solenoid {

    private boolean m_inverted = false;

    /**
     * Creates a Solenoid on the PCM (Pneumatic Control
     * Module) with CAN ID 0. If your PCM does not have
     * CAN ID 0, this will not work. Use
     * {@link #NKSolenoid(int,int)} instead.
     * @param channel The solenoid's channel on the PCM
     */
    public NKSolenoid(PneumaticsModuleType type, int channel) {
        this(0, type, channel);
    }

    /**
     * Creates a Solenoid on the PCM (Pneumatic Control
     * Module) with specified CAN ID.
     * @param moduleNumber The CAN ID for the PCM
     * @param channel The solenoid's channel on the PCM
     */
    public NKSolenoid(int moduleNumber, PneumaticsModuleType type, int channel) {
        super(moduleNumber, type, channel);
    }

    /**
     * Sets the Solenoid, taking into account whether
     * or not the solenoid is inverted.
     * @param on Set to true/false for extend/retract
     *           If these are flipped, you can call
     *           the {@link #invert()} method once 
     *           and that should fix it for you.
     */
    @Override
    public void set(boolean on) {
        if (!m_inverted && on != super.get()) {
            super.set(on);
        } else if (m_inverted && on == super.get()) {
            super.set(!on);
        }
    }

    /**
     * Returns the Solenoid's value, taking into account
     * whether or not the solenoid is inverted.
     */
    @Override
    public boolean get() {
        return m_inverted != super.get();
    }

    /**
     * Inverts the solenoid. If the solenoid is already
     * inverted, it will invert twice and become normal
     * again.
     */
    public void invert() {
        m_inverted = !m_inverted;
    }

    /**
     * Sets the solenoid inverted or not depending on
     * the boolean parameter you pass in.
     * @param invert true -> inverted, false -> normal
     */
    public void setInverted(boolean invert) {
        m_inverted = invert;
    }
}