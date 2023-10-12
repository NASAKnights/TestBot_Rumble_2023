// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Punch;
import frc.robot.commands.UnPunch;
import frc.robot.drive.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick driver;

  private AHRS navx;
  private SwerveDrive swerve;

  private DoubleSolenoid punchPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0); //TODO: Figure out these values
  private PneumaticHub pHub;

  public RobotContainer() {
    driver = new Joystick(Constants.kDriverPort);

    new JoystickButton(driver, 0).whileTrue(new RepeatCommand(new Punch(punchPiston))).onFalse(new UnPunch(punchPiston));
    //TODO: Change buttonNumber

    pHub = new PneumaticHub(2);
    pHub.enableCompressorAnalog(Constants.PneumaticConstants.kMinPressure, Constants.PneumaticConstants.kMaxPressure);//TODO: check these

    swerve = new SwerveDrive(navx);
        
    swerve.readoffsets();
    swerve.initDashboard();
    swerve.updateSmartDash();
  }

  public void disabledPeriodic(){
    swerve.updateSmartDash();
    swerve.writeOffsets();
    swerve.readoffsets();
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//TODO: Change this
  }
}
