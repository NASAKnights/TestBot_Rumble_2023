// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class AutoBalance extends CommandBase {

  private SwerveDrive swerve;
  private double balanceThreshold = 5;
  private Pose2d currentPose;
  private Timer timer;
  private double checkPeriod = 0.25;
  private Pose2d desiredPose;
  private double moveDistance = 0.035;
  // private double moveDis;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.initializePID();
    currentPose = swerve.getPose();
    desiredPose = swerve.getPose();
    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d moveBy;
    if(timer.advanceIfElapsed(checkPeriod)){
      SmartDashboard.putNumber("isTiming", desiredPose.getX());
      if(Math.abs(swerve.getPitch()) < balanceThreshold){
        desiredPose = swerve.getPose();
        new AutoRotateForDegrees(swerve, 1);
      }else if(swerve.getPitch() > balanceThreshold){
        moveBy = new Transform2d(new Translation2d(-moveDistance, 0), new Rotation2d());
        desiredPose = desiredPose.transformBy(moveBy);
      }else{
        moveBy = new Transform2d(new Translation2d(moveDistance, 0), new Rotation2d());
        desiredPose = desiredPose.transformBy(moveBy);
      }
    }
    swerve.setReference(desiredPose);
    // SmartDashboard.putNumber("roll", swerve.getRoll());
    // SmartDashboard.putNumber("pitch", swerve.getPitch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
