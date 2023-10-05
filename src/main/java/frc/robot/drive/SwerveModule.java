
package frc.robot.drive;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.motors.NKTalonFX;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private NKTalonFX drive;
    private NKTalonFX turn;
    private CANCoder turnEncoder;
    private Rotation2d angleOffset;
    private SimpleMotorFeedforward feedforward;

    private int id;

    private double lastAngle;

    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, Rotation2d angleOffset) {
        this.angleOffset = angleOffset;
        id = (driveMotorID / 10);

        SmartDashboard.getNumber("CanCoder ", driveMotorID + 2);
        initEncoder(encoderID);
        initDriveMotor(driveMotorID);
        initTurnMotor(turnMotorID);
        this.feedforward = new SimpleMotorFeedforward(ModuleConstants.kDriveS, ModuleConstants.kDriveV,
                ModuleConstants.kDriveA);

        resetToAbsolute();
        
    }

    public SwerveModuleState getCurrentState() {
        double velocity = getVelocityMPS();
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = getDistanceMeters();
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModulePosition(distance, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (turn.hasResetOccurred()) {
            resetToAbsolute();
        }

        Rotation2d currentAngleRotation2d = getAngleRotation2d();

        desiredState = SwerveModuleState.optimize(desiredState, currentAngleRotation2d);

        desiredState = checkForWrapAround(desiredState, currentAngleRotation2d);
        // SmartDashboard.putNumber(id + " angle states", desiredState.angle.getDegrees());


        // sets Angle and velocity of the wheels
        setAngle(desiredState, currentAngleRotation2d);
        setVelocity(desiredState, isOpenLoop);
    }

    public void resetDriveEncoders(){
        drive.resetPosition();
    }

    // sets angle for a module
    private void setAngle(SwerveModuleState desiredState, Rotation2d currentAngleRotation2d) {

        // calculate angle for the falcon to go to
        // double angle = 0.0;
        double angle = desiredState.angle.getDegrees();
        if (Math.abs(desiredState.speedMetersPerSecond) >= (ModuleConstants.kMaxSpeed * 0.01)) {
            // if moving, use the desired state's angle
            angle = desiredState.angle.getDegrees();
        } else {
            // if not moving, use the last given angle
            angle = lastAngle;
        }

        if (Math.abs(currentAngleRotation2d.minus(desiredState.angle)
                .getDegrees()) > ModuleConstants.kAllowableAngleTolerance.getDegrees()) {
            turn.set(ControlMode.Position, Conversions.degreesToFalcon(angle, ModuleConstants.kTurnGearRatio));
        } else {
            // don't move if angle is small
            turn.set(ControlMode.PercentOutput, 0);
        }
        lastAngle = angle;

    }

    // sets Velocity for a module
    private void setVelocity(SwerveModuleState desiredState, boolean isOpenLoop) {
        // set the drive velocity
        if (isOpenLoop) {
            // estimates percentage of motor and sets it
            double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kMaxSpeed;
            drive.set(ControlMode.PercentOutput, percentOutput);
        } else {
            // convert desired speed in m/s to falcon units
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    ModuleConstants.kWheelCircumference, ModuleConstants.kDriveGearRatio);

            // set velocity using feedforward
            drive.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        
    }


    private void resetToAbsolute() {
        lastAngle = (getCANCoder().minus(angleOffset).getDegrees());
        double absolutePosition = Conversions.degreesToFalcon(lastAngle, ModuleConstants.kTurnGearRatio);
        turn.setSelectedSensorPosition(absolutePosition);
        
        // System.out.println(id + " INIT ANGLE " + lastAngle);
        // System.out.println(id + " ANGLE OFFSET " + angleOffset.getDegrees());
        // System.out.println(id + " abs pos " + absolutePosition);
        // System.out.println(id + " selected sensor pos " + turn.getSelectedSensorPosition());
    }

    private void initDriveMotor(int driveMotorID) {
        drive = new NKTalonFX(driveMotorID);

        drive.configFactoryDefault();
        drive.configAllSettings(ModuleConstants.kDriveMotorConfig);
        drive.setInverted(ModuleConstants.kDriveMotorInverted);
        drive.setNeutralMode(ModuleConstants.kDriveMotorNeutral);
        drive.setSelectedSensorPosition(0);

    }

    private void initTurnMotor(int turnMotorID) {
        turn = new NKTalonFX(turnMotorID);
        turn.configFactoryDefault();
        turn.configAllSettings(ModuleConstants.kTurnMotorConfig);
        turn.setInverted(ModuleConstants.kTurnMotorInverted);
        // turn.setSensorPhase(true);
        turn.setNeutralMode(ModuleConstants.kTurnMotorNeutral);
        // turn.configRemoteFeedbackFilter(turnEncoder, 0);
        turn.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        // turn.setIntegralAccumulator(0);
        // turn.clearStickyFaults();
        
        resetToAbsolute();
    }

    private void initEncoder(int encoderID) {
        turnEncoder = new CANCoder(encoderID);

        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(ModuleConstants.kEncoderConfig);
        // turnEncoder.setPositionToAbsolute(); // not sure if needed
        // turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // turnEncoder.configSensorDirection(false);
        // turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        
        // turnEncoder.configMagnetOffset(-angleOffset.getDegrees()); //TODO: need to fix this function
    }

    public void updateSmartDash() {
        SmartDashboard.putNumber(id + "Last Angle", lastAngle);
        // SmartDashboard.putNumber(id + " Current Angle", getAngleRotation2d().getDegrees());
        // SmartDashboard.putNumber(id + " Module Encoder Raw Position", turnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber(id + " Motor Integrated Sensor Position", turn.getSelectedSensorPosition());
        SmartDashboard.putNumber(id + "Magnet offset", angleOffset.getDegrees());
        // SmartDashboard.putNumber(id + " Module Angle", turnEncoder.getPosition());
        // SmartDashboard.putNumber(id + " turn.getPos()", turn.getSelectedSensorPosition());
        // SmartDashboard.putNumber(id + " cancoder", turnEncoder.getAbsolutePosition());
    }

    private SwerveModuleState checkForWrapAround(SwerveModuleState desiredState, Rotation2d currentState){
        // This is to make sure that the motor only turns at most 90 degrees in any direction

        double desiredDegress = desiredState.angle.getDegrees();
        double speedMulti;
        
        double remainder = Math.IEEEremainder(desiredDegress - currentState.getDegrees(), 180);
        double newPos = remainder + currentState.getDegrees();
        double minDist = Math.abs(Math.IEEEremainder(newPos - desiredDegress, 180));
        if (minDist < 0.001){
            speedMulti = 1;
        }
        else{
            speedMulti = -1;
        }

        desiredState.angle = Rotation2d.fromDegrees(newPos);
        desiredState.speedMetersPerSecond *= speedMulti;

        return desiredState;

    }

    private Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());

    }

    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(turn.getSelectedSensorPosition(), ModuleConstants.kTurnGearRatio));
    }

    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());
    }

    public double getDistanceMeters() {
        double distance = Conversions.falconToMeters(drive.getSelectedSensorPosition(),
                ModuleConstants.kWheelCircumference, ModuleConstants.kDriveGearRatio);
        return distance;
    }

    public double getVelocityMPS() {
        return Conversions.falconToMPS(drive.getSelectedSensorVelocity(), ModuleConstants.kWheelCircumference,
                ModuleConstants.kDriveGearRatio);
    }

    public void setTurnCoast(){
        turn.setNeutralMode(NeutralMode.Coast);
    }

    public void setTurnBrake(){
        turn.setNeutralMode(NeutralMode.Brake);
    }

    public void setOffset(double offset){
        this.angleOffset = Rotation2d.fromDegrees(offset);
        resetToAbsolute();
    }

    public void testDriveSpinny(double output) {
        drive.set(output);
    }

    public void testTurnSpinny(double output) {
        turn.set(output);
    }

    public void testStopSpinny() {
        drive.set(0);
        turn.set(0);
    }

    public void setDriveVoltage(double voltage) {
        drive.setVoltage(voltage);
    }

    public static final class Conversions {

        public static final double kSecondsPerMinute = 60.0;
        public static final double kFalconTicks = 2048.0;
        public static final double kMaxFalconRPM = 6380.0;
        public static final double kFalconToDegrees = 360.0 / kFalconTicks;
        public static final double kFalconToRotations = 1.0 / kFalconTicks;

        // starting units: ticks/100ms
        // ticks/100ms * (1000ms/s * 60s/m) * 1 rot/ 2048 ticks = 600 ticks/m * 1 rot/ticks
        public static final double kFalconVelocityToRPM = 600.0 / kFalconTicks;

        public static double falconToDegrees(double counts, double gearRatio) {
            // ratio = motor/wheel
            return counts * kFalconToDegrees / gearRatio;
        }

        public static double degreesToFalcon(double degrees, double gearRatio) {
            double ticks = degrees * gearRatio / kFalconToDegrees;
            return ticks;
        }

        /**
         * Converts a falcon motor position into distance traveled
         * 
         * @param falconPosition falcon position sensor counts
         * @param circumference  wheel circumference in meters
         * @param gearRatio      motor rotations/wheel rotations
         * @return distance traveled in meters
         */
        public static double falconToMeters(double falconPosition, double circumference, double gearRatio) {
            double wheelRotations = falconPosition * kFalconToRotations / gearRatio;
            double distance = wheelRotations * circumference;
            return distance;
        }

        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * kFalconVelocityToRPM;
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM / kFalconVelocityToRPM;
            return sensorCounts;
        }

        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / kSecondsPerMinute;
            return wheelMPS;
        }

        public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
            double wheelRPM = ((velocity * kSecondsPerMinute) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }
    }
}
