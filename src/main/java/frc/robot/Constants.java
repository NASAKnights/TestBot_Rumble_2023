package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.lib.sds.SDSModuleType;

public class Constants {

    public static final boolean kDemoMode = true;

    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;
    public static final SPI.Port kNavXPort = SPI.Port.kMXP;

    public static final SDSModuleType kSDSModule = SDSModuleType.MK4I_L3;

    public static final double kTrackwidthMeters = 0.4;
    public static final double kWheelbaseMeters = 0.4;

    public static final double kDefaultAxisDeadband = 0.15;

    // rpm / (60 s / min) * (g * C m / rot) where g is the gear ratio and C is the
    // curcumference of the wheel
    /**
     * Maximum translational velocity of the robot in meters per second
     */
    public static final double kMaxTranslationalVelocity = ((6380.0) / 60) * kSDSModule.getWheelDiameter() * Math.PI
            * kSDSModule.getDriveReduction();
    /**
     * Maximum rotational velocity of the robot
     */
    public static final double kMaxRotationalVelocity = (kMaxTranslationalVelocity)
            / Math.hypot(kTrackwidthMeters / 2, kWheelbaseMeters / 2);
    public static final boolean kIsFieldRelative = true;
    public static final boolean kIsOpenLoop = false;

    public static final int kFrontLeftDriveMotorID = 10;
    public static final int kFrontLeftTurnMotorID = 11;
    public static final int kFrontLeftEncoderID = 12;
    public static final Rotation2d kFrontLeftOffset = Rotation2d.fromDegrees(103.711); // module 1
    public static final Translation2d kFrontLeftPosition = new Translation2d(kTrackwidthMeters / 2.0,
            kWheelbaseMeters / 2.0);

    public static final int kFrontRightDriveMotorID = 20;
    public static final int kFrontRightTurnMotorID = 21;
    public static final int kFrontRightEncoderID = 22;
    public static final Rotation2d kFrontRightOffset = Rotation2d.fromDegrees(318.427); // 139.658 // 139.658 // module 2
    public static final Translation2d kFrontRightPosition = new Translation2d(kTrackwidthMeters / 2.0,
            -kWheelbaseMeters / 2.0);

    public static final int kBackLeftDriveMotorID = 30;
    public static final int kBackLeftTurnMotorID = 31;
    public static final int kBackLeftEncoderID = 32;
    public static final Rotation2d kBackLeftOffset = Rotation2d.fromDegrees(275.273);  // 95.977 // 96.680 // module 3
    public static final Translation2d kBackLeftPosition = new Translation2d(-kTrackwidthMeters / 2.0,
            kWheelbaseMeters / 2.0);

    public static final int kBackRightDriveMotorID = 40;
    public static final int kBackRightTurnMotorID = 41;
    public static final int kBackRightEncoderID = 42;
    public static final Rotation2d kBackRightOffset = Rotation2d.fromDegrees(87.539); // 265.517 // -93.867 // module 4
    public static final Translation2d kBackRightPosition = new Translation2d(-kTrackwidthMeters / 2.0,
            -kWheelbaseMeters / 2.0);

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(kFrontLeftPosition,
            kFrontRightPosition, kBackLeftPosition, kBackRightPosition);

    //-----------testing--------------------

    // select color and then version
    public static final double[][] encoderoffsets = {{358.188 - 25.0 - 8.0 - 14.0 - 5.0 - 13.0, 103.711, 0.0}, // Red 1, 5, 9
                                                    {330.654 + 7 - 3.5, 318.427,  0.0}, // Blue 2, 6, 10
                                                    {257.607, 275.273, 0.0}, // green 3, 7, 11
                                                    {103.359 - 5.0 - 4.0, 87.539,  0.0}}; // yellow 4, 8, 12

    public static double kDriveLimit = kDemoMode? 0.4 : 0.5; // 0.7 fast
    public static double kRotationLimit = kDriveLimit;



    //-----------testing end----------------

    public static final class ModuleConstants {

        public static final double kMaxSpeed = frc.robot.Constants.kMaxTranslationalVelocity;
        public static final double kWheelDiameterMeters = kSDSModule.getWheelDiameter();
        public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
        // ratio is motor rot / wheel rot
        public static final double kDriveGearRatio = 1 / kSDSModule.getDriveReduction();
        public static final double kTurnGearRatio = 1 / kSDSModule.getSteerReduction();

        public static final boolean kDriveMotorInverted = kSDSModule.isDriveInverted();
        public static final NeutralMode kDriveMotorNeutral = NeutralMode.Coast;
        public static final double kDriveVoltageComp = 12;
        public static final boolean kTurnMotorInverted = true;
        public static final NeutralMode kTurnMotorNeutral = NeutralMode.Brake; // set back to brake to be amazing
        public static final double kTurnVoltageComp = 12;
        // later
        public static final boolean kEncoderInverted = false;

        public static final TalonFXConfiguration kDriveMotorConfig = new TalonFXConfiguration() {
            {
                this.slot0.kP = kDriveP;
                this.slot0.kI = kDriveI;
                this.slot0.kD = kDriveD;
                this.slot0.kF = kDriveF;
                this.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                        kDriveEnableCurrentLimit,
                        kDriveContinuousCurrentLimit,
                        kDrivePeakCurrentLimit,
                        kDrivePeakCurrentDuration);

                this.initializationStrategy = SensorInitializationStrategy.BootToZero;
                this.openloopRamp = kOpenLoopRamp;
                this.closedloopRamp = kClosedLoopRamp;
                this.voltageCompSaturation = kDriveVoltageComp;
                this.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;

            }
        };
        public static final TalonFXConfiguration kTurnMotorConfig = new TalonFXConfiguration() {
            {
                this.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
                this.primaryPID.selectedFeedbackCoefficient = 1;
                this.slot0.kP = kTurnP;
                this.slot0.kI = kTurnI;
                this.slot0.kD = kTurnD;
                this.slot0.kF = kTurnF;
                this.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                        kTurnEnableCurrentLimit,
                        kTurnContinuousCurrentLimit,
                        kTurnPeakCurrentLimit,
                        kTurnPeakCurrentDuration);

                this.initializationStrategy = SensorInitializationStrategy.BootToZero;
                this.voltageCompSaturation = kTurnVoltageComp;
            }
        };
        public static final CANCoderConfiguration kEncoderConfig = new CANCoderConfiguration() {
            {
                this.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                this.sensorDirection = kEncoderInverted;
                this.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                this.sensorTimeBase = SensorTimeBase.PerSecond;
            }
        };

        public static final double kOpenLoopRamp = 0.50;
        public static final double kClosedLoopRamp = 0.50;

        public static final boolean kTurnEnableCurrentLimit = true;
        public static final int kTurnContinuousCurrentLimit = 25;
        public static final int kTurnPeakCurrentLimit = 40;
        public static final double kTurnPeakCurrentDuration = 0.1;

        public static final boolean kDriveEnableCurrentLimit = true;
        public static final int kDriveContinuousCurrentLimit = 35;
        public static final int kDrivePeakCurrentLimit = 60;
        public static final double kDrivePeakCurrentDuration = 0.1;

        public static final double kDriveS = 0.667 / 12.0;
        public static final double kDriveV = 2.44 / 12.0;
        public static final double kDriveA = 0.27 / 12.0;

        public static final double kDriveP = 0.10;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveF = 0; // 0.25 / 0.54 * 0.1;

        public static final double kTurnP = 0.6;
        public static final double kTurnI = 0;
        public static final double kTurnD = 12.0; // 12.0
        public static final double kTurnF = 0.0;

        public static final Rotation2d kAllowableAngleTolerance = Rotation2d.fromDegrees(5.0);
    }

    // Rapid React constants ---------------------------------------------------

    public static final int kPDH = 40;
    public static final ModuleType kPDHType = ModuleType.kRev;
    public static final int kPH = 41;
    public static final PneumaticsModuleType kPHType = PneumaticsModuleType.REVPH;

    public static final class OIConstants {
        // controller ID constants
        public static final int kDriverID = 0;
        public static final int kOperatorID = 1;

        // joystick button numbers
        public static final int kSquare = 1;
        public static final int kX = 2;
        public static final int kCircle = 3;
        public static final int kTriangle = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftTriggerButton = 7;
        public static final int kRightTriggerButton = 8;
        public static final int kShare = 9;
        public static final int kOptions = 10;
        public static final int kLeftStickButton = 11;
        public static final int kRightStickButton = 12;

        // joystick axis numbers
        public static final int kLeftXJoystick = 0;
        public static final int kLeftYJoystick = 1;
        public static final int kRightXJoystick = 2;
        public static final int kRightYJoystick = 5;
        public static final int kLeftTrigger = 3;
        public static final int kRightTrigger = 4;
    }


    public static class IntakeConstants {
        // Motor CAN IDs
        public static final byte kIntakeMotorID = 9;

        // Solenoid Pneumatic Hub Channels
        public static final byte kIntakeDeployerForwardChannel = 2;
        public static final byte kIntakeDeployerReverseChannel = 3;

        // Acceptable speeds for intake
        public static final double kMinIntakeSpeed = 0.25;
        public static final double kMaxIntakeSpeed = 0.60;
        public static final double kDefaultIntakeSpeed = 0.50;

        // Values for intake positions
        public static final DoubleSolenoid.Value kRetracted = Value.kReverse;
        public static final DoubleSolenoid.Value kDeployed = Value.kForward;
    }

    public static class ShooterConstants {
        // Motor CAN ID
        public static final byte kFlywheelMotorID = 7;

        // Solenoid Pneumatic Hub Channels
        public static final byte kHoodForwardChannelID = 0;
        public static final byte kHoodReverseChannelID = 1;


        // RoboRio Sensor Ports
        public static final byte kShooterSwitchID = 0;
        public static final I2C.Port kColorSensorPort = I2C.Port.kOnboard;


        // Solenoid Values
        public static final DoubleSolenoid.Value kHoodRetracted = Value.kReverse;
        public static final DoubleSolenoid.Value kHoodExtended = Value.kForward;

        // Opposite Alliance Cargo Shots
        public static final boolean kOppositeColorHoodExtended = false; // TODO Find this
        public static final double kOppositeColorRPM = 800; // TODO Find this

        // Autonomous Cargo Shots
        public static final boolean kTARMACEdgeHoodExtended = false;
        public static final double kTARMACEdgeShotRPM = 3800;


        // Tranfer wheel speeds
        public static final double kDefaultTransferSpeed = 0.40; // TODO Find this

        // Tolerances on the flywheel
        public static final double kVelocityToleranceRPM = 40;
        public static final int kMinCount = 4;

        public static final boolean kAutoShotHoodExtended = true;
		public static final double kAutoShotRPM = 3650;

        public static final boolean kSafeZoneHoodExtended = true;
		public static final double kSafeZoneRPM = 4300;

        public static final boolean kFlywheelEnableCurrentLimit = true;
        public static final int kFlywheelContinuousCurrentLimit = 25;
        public static final int kFlywheelPeakCurrentLimit = 30;
        public static final double kFlywheelPeakCurrentDuration = 0.1;

        // Flywheel configs
        public static final TalonFXConfiguration kFlywheelMotorConfig = new TalonFXConfiguration() {
            {
                this.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
                this.primaryPID.selectedFeedbackCoefficient = 1;
                this.slot0.kP = 0.15;
                this.slot0.kI = 0.0;
                this.slot0.kD = 0.0;
                this.slot0.kF = 0.0480;

                this.voltageCompSaturation = 12.0;
                this.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                        kFlywheelEnableCurrentLimit,
                        kFlywheelContinuousCurrentLimit,
                        kFlywheelPeakCurrentLimit,
                        kFlywheelPeakCurrentDuration);
                
            }
        };
    }
    public static final class PneumaticConstants{

        public static final int kPneumaticHubModule = 2;

        public static final int kAnalogPressureChannel = 1;

        public static final double kMinPressure = 85.0;
        public static final double kMaxPressure = 110.0;

        public static final int kSolenoidForward = 0;
        public static final int kSolenoidReverse = 1;
    }
}
