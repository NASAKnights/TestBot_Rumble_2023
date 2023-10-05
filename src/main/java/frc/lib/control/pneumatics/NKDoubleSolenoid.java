package frc.lib.control.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class NKDoubleSolenoid extends DoubleSolenoid {

    public NKDoubleSolenoid(PneumaticsModuleType type, int forwardChannel, int reverseChannel) {
        this(0, type, forwardChannel, reverseChannel);
    }

    public NKDoubleSolenoid(int moduleNumber, PneumaticsModuleType type, int forwardChannel, int reverseChannel) {
        super(moduleNumber, type, forwardChannel, reverseChannel);
    }

    @Override
    public void set(Value value) {
        if (value != this.get())
         super.set(value);
    }

}