package frc.robot.drive.commands;

import frc.robot.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.drive.SwerveDrive;

public class Drive extends CommandBase {

    private Joystick driver;
    private SwerveDrive swerve;

    public Drive(Joystick driver, SwerveDrive swerve) {
        this.driver = driver;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {

        double limit = Constants.kDriveLimit;
        double rotLimit = Constants.kRotationLimit;
        double xSpeed = -MathUtil.calculateAxis(driver.getY(), Constants.kDefaultAxisDeadband, limit * Constants.kMaxTranslationalVelocity);
        double ySpeed = -MathUtil.calculateAxis(driver.getX(), Constants.kDefaultAxisDeadband, limit * Constants.kMaxTranslationalVelocity);
        double thetaSpeed = -MathUtil.calculateAxis(driver.getZ(), Constants.kDefaultAxisDeadband, rotLimit * Constants.kMaxRotationalVelocity);

        // SmartDashboard.putNumber("z", driver.getZ()); 
        // SmartDashboard.putNumber("theta", thetaSpeed);
        // SmartDashboard.putNumber("rot", Constants.MAX_ROTATIONAL_VELOCITY);
        // System.out.println(Constants.MAX_ROTATIONAL_VELOCITY);
        ChassisSpeeds speeds;

        if (Constants.kIsFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, swerve.getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        }

        // System.out.println("DriveCommand(" + xSpeed + ", " + ySpeed + ", " + thetaSpeed + ")");
        swerve.drive(speeds, Constants.kIsOpenLoop);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(), Constants.kIsOpenLoop);
    }
}
