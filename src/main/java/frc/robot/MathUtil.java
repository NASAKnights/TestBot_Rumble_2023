package frc.robot;

public class MathUtil {
    public static double calculateAxis(double axis, double deadband, double scalar) {
        double res;

        if (Math.abs(axis) > deadband) {
            if (axis > 0.0) {
                res = (axis - deadband) / (1.0 - deadband);
            } else {
                res = (axis + deadband) / (1.0 - deadband);
            }
        } else {
            res = 0.0;
        }

        return res * scalar;
    }
}
