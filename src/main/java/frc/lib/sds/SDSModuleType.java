package frc.lib.sds;

/**
 * SDS module information copied for robot code because there (currently) isn't
 * a version that works with WPILib 2023
 */
public enum SDSModuleType {

    // mk3
    MK3_STANDARD(
            0.1016,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true),
    MK3_FAST(
            0.1016,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true),

    // mk4
    MK4_L1(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true),
    MK4_L2(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true),
    MK4_L3(
            0.10033,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true),
    MK4_L4(
            0.10033,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true),

    // mk4i
    MK4I_L1(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false),
    MK4I_L2(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false),
    MK4I_L3(
            0.10033,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false);

    private final double wheelDiameter;
    private final double driveReduction;
    private final boolean driveInverted;

    private final double steerReduction;
    private final boolean steerInverted;

    /**
     * @param wheelDiameter  The diameter of the module's wheel in meters.
     * @param driveReduction The overall drive reduction of the module. Multiplying
     *                       motor rotations by this value
     *                       should result in wheel rotations.
     * @param driveInverted  Whether the drive motor should be inverted. If there is
     *                       an odd number of gea reductions
     *                       this is typically true.
     * @param steerReduction The overall steer reduction of the module. Multiplying
     *                       motor rotations by this value
     *                       should result in rotations of the steering pulley.
     * @param steerInverted  Whether the steer motor should be inverted. If there is
     *                       an odd number of gear reductions
     *                       this is typically true.
     */
    SDSModuleType(double wheelDiameter, double driveReduction, boolean driveInverted,
            double steerReduction, boolean steerInverted) {
        this.wheelDiameter = wheelDiameter;
        this.driveReduction = driveReduction;
        this.driveInverted = driveInverted;
        this.steerReduction = steerReduction;
        this.steerInverted = steerInverted;
    }

    /**
     * Gets the diameter of the wheel in meters.
     */
    public double getWheelDiameter() {
        return wheelDiameter;
    }

    /**
     * Gets the overall reduction of the drive system.
     * <p>
     * If this value is multiplied by drive motor rotations the result would be
     * drive wheel rotations.
     */
    public double getDriveReduction() {
        return driveReduction;
    }

    /**
     * Gets if the drive motor should be inverted.
     */
    public boolean isDriveInverted() {
        return driveInverted;
    }

    /**
     * Gets the overall reduction of the steer system.
     * <p>
     * If this value is multiplied by steering motor rotations the result would be
     * steering pulley rotations.
     */
    public double getSteerReduction() {
        return steerReduction;
    }

    /**
     * Gets if the steering motor should be inverted.
     */
    public boolean isSteerInverted() {
        return steerInverted;
    }

    @Override
    public String toString() {

        StringBuilder builder = new StringBuilder();

        builder.append("SDSModuleType{");
        builder.append("name=");
        builder.append(this.name());
        builder.append(", wheelDiameter=");
        builder.append(wheelDiameter);
        builder.append(", driveReduction=");
        builder.append(driveReduction);
        builder.append(", driveInverted=");
        builder.append(driveInverted);
        builder.append(", steerReduction=");
        builder.append(steerReduction);
        builder.append(", steerInverted=");
        builder.append(steerInverted);
        builder.append('}');

        return builder.toString();
    }
}
