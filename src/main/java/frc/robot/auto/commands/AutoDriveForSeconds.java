// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class AutoDriveForSeconds extends CommandBase {
  
  private SwerveDrive swerve;
  private ChassisSpeeds speeds;
  private double seconds;
  private Timer timer;

  public AutoDriveForSeconds(SwerveDrive swerve, ChassisSpeeds speeds, double seconds) {
    this.swerve = swerve;
    this.speeds = speeds;
    this.seconds = seconds;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < seconds){
      swerve.drive(speeds, false);
    }else{
      end(isFinished());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() < seconds){
    return false;
    }else{
      return true;
    }
  }
}
