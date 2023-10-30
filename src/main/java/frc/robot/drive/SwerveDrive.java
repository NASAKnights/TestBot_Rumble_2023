package frc.robot.drive;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveDrive extends SubsystemBase {

    private AHRS navx;

    private ChassisSpeeds speeds;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private SwerveModule[] modules;

    private int red, blue, green, yellow;

    private PIDController pidX, pidY, pidRot;

    private boolean hasRun;

    public SwerveDrive(AHRS navx) {
        this.navx = navx;
        this.navx.calibrate();
        this.navx.setAngleAdjustment(90);

        this.speeds = new ChassisSpeeds();
        this.kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(Constants.kTrackwidthMeters / 2.0,
                                Constants.kWheelbaseMeters / 2.0),
                // Front right
                new Translation2d(Constants.kTrackwidthMeters / 2.0,
                                -Constants.kWheelbaseMeters / 2.0),
                // Back left
                new Translation2d(-Constants.kTrackwidthMeters / 2.0,
                                Constants.kWheelbaseMeters / 2.0),
                // Back right
                new Translation2d(-Constants.kTrackwidthMeters / 2.0,
                                -Constants.kWheelbaseMeters / 2.0));

        this.frontLeft = new SwerveModule(
                Constants.kFrontLeftDriveMotorID,
                Constants.kFrontLeftTurnMotorID,
                Constants.kFrontLeftEncoderID,
                Constants.kFrontLeftOffset);

        this.frontRight = new SwerveModule(
                Constants.kFrontRightDriveMotorID,
                Constants.kFrontRightTurnMotorID,
                Constants.kFrontRightEncoderID,
                Constants.kFrontRightOffset);

        this.backLeft = new SwerveModule(
                Constants.kBackLeftDriveMotorID,
                Constants.kBackLeftTurnMotorID,
                Constants.kBackLeftEncoderID,
                Constants.kBackLeftOffset);

        this.backRight = new SwerveModule(
                Constants.kBackRightDriveMotorID,
                Constants.kBackRightTurnMotorID,
                Constants.kBackRightEncoderID,
                Constants.kBackRightOffset);

                this.modules = new SwerveModule[] { this.frontLeft, this.frontRight, this.backLeft, this.backRight };

                this.odometry = new SwerveDriveOdometry(this.kinematics, this.getHeading(), this.getModulePositions());

        readoffsets();
        updateOffsets();
        // System.out.println("Data: "+red+" "+yellow);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, false);
    }

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        this.speeds = speeds;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        // states[0].speedMetersPerSecond *= -1;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.kMaxSpeed);
        
        for (int i = 0; i < 4; i++) {
            this.modules[i].setDesiredState(states[i], isOpenLoop);
        }
    }

    public double getDistanceMeters(){
        return frontLeft.getDistanceMeters();
    }

    public void setCoast(){
        for (int i = 0; i < 4; i++){
            this.modules[i].setTurnCoast();
        }
    }

    public void setBrake(){
        for (int i = 0; i < 4; i++){
            this.modules[i].setTurnBrake();
        }
    }

    public void setTurboSpeed(){
        Constants.kDriveLimit = 0.8;

    }
    public void setSlowSpeed(){
        Constants.kDriveLimit = 0.3;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-navx.getAngle());
    }

    public void resetHeading() {
        navx.zeroYaw();
    }

    public void invertHeading(){
        navx.setAngleAdjustment(180);
    }

    public void invertHeading90(){
        navx.setAngleAdjustment(90);
    }
    public void invertHeading270() {
        navx.setAngleAdjustment(270);
    }

    public void resetHeadingOffset(){
        navx.setAngleAdjustment(0);
    }

    public void resetDriveEncoders(){
        for (int i = 0; i < 4; i ++){
            this.modules[i].resetDriveEncoders();
        }
    }

    public void initDashboard(){
        SmartDashboard.putNumber("Red encoder", red);
        SmartDashboard.putNumber("Blue encoder", blue);
        SmartDashboard.putNumber("Green encoder", green);
        SmartDashboard.putNumber("Yellow encoder", yellow);
    }

    public void updateSmartDash() {
        for (SwerveModule module : this.modules) {
                module.updateSmartDash();
        }
        SmartDashboard.putNumber("heading", this.getHeading().getDegrees());
        SmartDashboard.putNumber("x", this.odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", this.odometry.getPoseMeters().getY());
        
        red = (int) SmartDashboard.getNumber("Red encoder", 0);
        blue = (int) SmartDashboard.getNumber("Blue encoder", 0);
        green = (int) SmartDashboard.getNumber("Green encoder", 0);
        yellow = (int) SmartDashboard.getNumber("Yellow encoder", 0);

        SmartDashboard.putNumber("roll", navx.getRoll());
        SmartDashboard.putNumber("pitch", navx.getPitch());
        SmartDashboard.putNumber("yaw", navx.getYaw());

        
    }

    public void readoffsets(){
        try {
            FileReader reader = new FileReader("/home/lvuser/OffsetConfig.txt");
            // int character;
            char[] buffer = new char[4];
 
            if (reader.read(buffer, 0, 4) != 4){
                // System.out.println("Inside the if " + reader.read(buffer, 0, 4));
                buffer[0] = '0';
                buffer[1] = '0';
                buffer[2] = '0';
                buffer[3] = '0';
                // System.out.println("ENTERED INTO THE NOT READING");
            }

            this.red = buffer[0] - '0';
            this.blue = buffer[1] - '0';
            this.green = buffer[2] - '0';
            this.yellow = buffer[3] - '0';

            reader.close();

            // in case we put an out of bounds number
            if (this.red > 2 || this.red < 0){
                this.red = 0;
            }
            if (this.blue > 2 || this.blue < 0){
                this.blue = 0;
            }
            if (this.green > 2 || this.green < 0){
                this.green = 0;
            }
            if (this.yellow > 2 || this.yellow < 0){
                this.yellow = 0;
            }
            // System.out.println("Reading, READING");
            // System.out.println(buffer);
 
        } catch (IOException e) {
            e.printStackTrace();
        }
        // System.out.println(red);
        // System.out.println("REDREDREDREDREDRED");
    }

    public void writeOffsets(){
        try {
            FileWriter writer = new FileWriter("/home/lvuser/OffsetConfig.txt", false);
            String s = "";
            s += this.red;
            s += this.blue;
            s += this.green;
            s += this.yellow;
            // s += "\n"; // idk if needed...
            writer.write(s);
            writer.close();

            // System.out.println(this.red);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public SwerveModule[] getModules() {
        return this.modules;
    }

    public void updateOffsets(){

        this.frontLeft.setOffset(Constants.encoderoffsets[0][red]);
        this.frontRight.setOffset(Constants.encoderoffsets[1][blue]);
        this.backLeft.setOffset(Constants.encoderoffsets[2][green]);
        this.backRight.setOffset(Constants.encoderoffsets[3][yellow]);

    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public void resetPose(Pose2d position) {
        this.odometry.resetPosition(this.getHeading(), this.getModulePositions(), position);
    }

    public Pose2d getPose(){
        return this.odometry.getPoseMeters();
    }

    private void updateOdometry() {
        // SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        this.odometry.update(this.getHeading(), this.getModulePositions());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getPitch(){
        return navx.getPitch();
    }

    public double getRoll(){
        return navx.getRoll();
    }



    public void initializePID(){
        pidX = new PIDController(0.9, 1e-4, 0);
        pidY = new PIDController(0.9, 1e-4, 0);
        pidRot = new PIDController(0.15, 0, 0);
        pidX.setTolerance(0.025); 
        pidY.setTolerance(0.025);
        pidRot.setTolerance(1);

        hasRun = false;
    }

    public void setReference(Pose2d pose){
        if(!pidX.atSetpoint() & !pidY.atSetpoint() | !hasRun){
            speeds = new ChassisSpeeds(
            pidX.calculate(getPose().getX(), pose.getX()),
            pidY.calculate(getPose().getY(), pose.getY()),
            0
            // pidRot.calculate(getPose().getRotation().getDegrees(), pose.getRotation().getDegrees())
          );
          
            drive(speeds, false);
            SmartDashboard.putNumber("x calculate", pidX.calculate(getPose().getX(), pose.getX()));
        }
        
    }


    @Override
    public void periodic() {
        this.updateOdometry();
        this.updateSmartDash();

    }

}
