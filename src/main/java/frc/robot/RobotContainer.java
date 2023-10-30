// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.commands.AutoTest;
import frc.robot.auto.commands.AutoWon;
import frc.robot.blower.Blower;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.ToggleSlow;
import frc.robot.drive.commands.ToggleTurbo;
import frc.robot.puncher.Puncher;
import frc.robot.puncher.commands.Punch;
import frc.robot.puncher.commands.UnPunch;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick driver;
  private Joystick operator;

  private Puncher punchie;
  private AutoWon newBalance;
  private AutoTest autoTest;

  private AHRS navx;
  private SwerveDrive swerve;
  private Blower blower;

  private PneumaticHub pHub;

  public RobotContainer() {
    driver = new Joystick(Constants.kDriverPort);
    operator = new Joystick(Constants.kOperatorPort);
    blower = new Blower();

    navx = new AHRS(Constants.kNavXPort);

    pHub = new PneumaticHub(1);
    pHub.enableCompressorAnalog(Constants.PneumaticConstants.kMinPressure, Constants.PneumaticConstants.kMaxPressure);//TODO: check these
    punchie = new Puncher();

    swerve = new SwerveDrive(navx);
        
    // swerve.readoffsets();
    swerve.initDashboard();
    swerve.updateSmartDash();


    swerve.setDefaultCommand(new DriveCommand(driver, swerve));
    configureButtonBindings();
    newBalance = new AutoWon(swerve, punchie);
    autoTest = new AutoTest(swerve);
  }

  public void disabledPeriodic(){
    swerve.updateSmartDash();
    // swerve.writeOffsets();
    // swerve.readoffsets();
    SmartDashboard.putNumber("Pressure", pHub.getPressure(0));
  }

  private void configureButtonBindings() {
    // Driver reset heading
    new JoystickButton(driver, 1).onTrue(new InstantCommand(swerve::resetHeading));

    // add a slow and fast button??
    new JoystickButton(driver, 5).onFalse(new ToggleSlow(swerve))
                                               .onTrue(new ToggleTurbo(swerve));

    if (Constants.kOnePlayer) {
      new JoystickButton(driver, 2).onTrue(new Punch(punchie)).onFalse(new UnPunch(punchie));
      new JoystickButton(driver, 3).onTrue(new InstantCommand(blower::start)).onFalse(new InstantCommand(blower::stop));
    }
    else {
      new JoystickButton(operator, 1).onTrue(new Punch(punchie)).onFalse(new UnPunch(punchie));
      new JoystickButton(operator, 3).onTrue(new InstantCommand(blower::start)).onFalse(new InstantCommand(blower::stop));
    }

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return autoTest;
    return newBalance;
  }

  public void invertHeading270() {
    swerve.invertHeading270();
  }

  public void invertHeading90() {
    swerve.invertHeading90();
  }

  public void resetHeading() {
    swerve.resetHeading();
  }
}
