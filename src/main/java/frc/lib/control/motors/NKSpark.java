package frc.lib.control.motors;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class NKSpark extends Spark {

    private double m_lastSpeed = 0;
    private double m_lastVoltage = 0;

    public NKSpark(int channel) {
        super(channel);
    }

    @Override
    public void set(double speed) {
        if (m_lastSpeed != speed) {
            m_lastSpeed = speed;
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) {
        if (this.m_lastVoltage != outputVolts) {
            this.m_lastVoltage = outputVolts;
            super.setVoltage(outputVolts);
        }
    }
    
}
