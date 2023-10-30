// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.puncher.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.puncher.Puncher;

public class UnPunch extends CommandBase {
  /** Creates a new UnPunch. */
  Puncher punchie;
  public UnPunch(Puncher punchie) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.punchPiston = punchPiston;
    this.punchie = punchie;
    addRequirements(punchie);
  }
  // Boolean finished = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    punchie.unpunch();
    // finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return finished;
    return true;
  }
}
