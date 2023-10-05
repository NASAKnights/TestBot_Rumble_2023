package frc.lib.control.motors;

import static frc.lib.control.motors.NKTalonFX.Math.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;

public class NKTalonFX extends WPI_TalonFX {

    public static final double kMaxFreeSpeedVelocityTicksPer100Milliseconds = 21777;
    public static final double kMaxFreeSpeedVelocityRPM = 6380;
    public static final double kTicksPerRotation = 2048;

    private ControlMode m_lastMode;
    private double m_lastOutput;
    protected int m_id;

    public NKTalonFX(int deviceNumber) {
        super(deviceNumber);
        super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
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

    public void setVelocity(double ticksPer100Milliseconds) {
        this.set(
            ControlMode.Velocity,
            MathUtil.clamp(
                ticksPer100Milliseconds,
                -NKTalonFX.kMaxFreeSpeedVelocityTicksPer100Milliseconds,
                NKTalonFX.kMaxFreeSpeedVelocityTicksPer100Milliseconds
            )
        );
    }

    public void setVelocityRPM(double rpm) {
        this.setVelocity(rpmToTicksPer100Milliseconds(rpm));
        // this.set(ControlMode.Velocity, rpmToTicksPer100Milliseconds(rpm));
    }

    public double getPosition() {
        return super.getSelectedSensorPosition();
    }

    public double getPositionRotations() {
        return this.getPosition() / NKTalonFX.kTicksPerRotation;
    }

    public double getAbsolutePosition() {
        return super.getSensorCollection().getIntegratedSensorAbsolutePosition();
    }

    public double getAbsolutePositionRotations() {
        return this.getAbsolutePosition() / NKTalonFX.kTicksPerRotation;
    }

    public double getVelocity() {
        return super.getSelectedSensorVelocity();
    }

    public double getVelocityRPM() {
        return ticksPer100MillisecondsToRPM(this.getVelocity());
    }

    public ErrorCode resetPosition() {
        return this.setPosition(0, 0);
    }

    public ErrorCode setPosition(double newPosition, int timeoutMs) {
        return super.setSelectedSensorPosition(newPosition, 0, timeoutMs);
    }

    public ErrorCode setPosition(double newPosition, int pidSlot, int timeoutMs) {
        return super.setSelectedSensorPosition(newPosition, pidSlot, timeoutMs);
    }

    public static class Math {

        /**
         * A rough estimate of the percent output
         * for a certain velocity in ticks per
         * 100 ms (default for ControlMode.Velocity)
         * @param ticksPer100Milliseconds velocity
         * @return the rough estimate
         */
        public static double ticksPer100MillisecondsToPercent(double ticksPer100Milliseconds) {
            return ticksPer100Milliseconds / NKTalonFX.kMaxFreeSpeedVelocityTicksPer100Milliseconds;
        }

        /**
         * A rough estimate of the percent output
         * for a certain velocity in RPM
         * @param ticksPer100Milliseconds velocity
         * @return the rough estimate
         */
        public static double rpmToPercent(double rpm) {
            return rpm / NKTalonFX.kMaxFreeSpeedVelocityRPM;
        }

        /**
         * Converts RPM to ticks per 100 ms.
         * @param rpm
         * @return ticks per 100 ms
         */
        public static double rpmToTicksPer100Milliseconds(double rpm) {
            return rpm * 3.41333333333;
        }

        /**
         * Converts ticks per 100 ms to RPM.
         * @param ticksPer100Milliseconds
         * @return RPM
         */
        public static double ticksPer100MillisecondsToRPM(double ticksPer100Milliseconds) {
            return ticksPer100Milliseconds / 3.41333333333;
        }
    }
}