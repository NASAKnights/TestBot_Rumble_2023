// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.drive.SwerveDrive;
import frc.robot.puncher.Puncher;
import frc.robot.puncher.commands.Punch;
import frc.robot.puncher.commands.UnPunch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoWon extends SequentialCommandGroup {
  /** Creates a new AutoScoreHighBalance. */
  private SwerveDrive swerve;
  private Puncher punchie;
  public AutoWon(SwerveDrive swerve, Puncher punchie) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.punchie = punchie;
    this.swerve = swerve;
  
    swerve.resetHeading();
    addCommands(
      new Punch(punchie),
      new UnPunch(punchie),
      // new InstantCommand(swerve::invertHeading90),
      new AutoDriveForSeconds(swerve, new ChassisSpeeds(0,-1.5,0), 1.5),
      // new AutoDriveForDistance(swerve, -1, 0, new Rotation2d(0)),
      new AutoBalance(swerve)
      // new InstantCommand(swerve::invertHeading270)
    );
  }
}
