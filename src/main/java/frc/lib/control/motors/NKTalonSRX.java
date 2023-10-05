package frc.lib.control.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class NKTalonSRX extends WPI_TalonSRX {
    /**
     * The default slot for PID control is zero.
     * If you would like to run more than one PID
     * loop on this motor, you may create a integer
     * with the number you want for your slot and
     * pass that in instead of this.
     * For the purposes of this program, we will use
     * the default PID slot. DO NOT CREATE A SLOT
     * WITH A VALUE OF ZERO! I have also created some
     * other PID slot ID numbers for you that do not
     * conflict with each other.
     * @see NKTalonFX#kSecondaryPIDSlot
     * @see NKTalonFX#kTertiaryPIDSlot
     * @see NKTalonFX#kQuaternaryPIDSlot
     */
    public static final int kDefaultPIDSlot = 0;

    public static final int kSecondaryPIDSlot = 1;
    public static final int kTertiaryPIDSlot = 2;
    public static final int kQuaternaryPIDSlot = 3;

    private ControlMode m_lastMode;
    private double m_lastOutput;

    public NKTalonSRX(int deviceNumber) {
        super(deviceNumber);
        this.m_handle = deviceNumber;
    }

    @Override
    public void set(ControlMode mode, double output) {
        if (mode != m_lastMode || output != m_lastOutput) {
            m_lastMode = mode;
            m_lastOutput = output;
            super.set(mode, output);
        }
    }

    public void set(double power) {
        this.set(ControlMode.PercentOutput, power);
    }
}