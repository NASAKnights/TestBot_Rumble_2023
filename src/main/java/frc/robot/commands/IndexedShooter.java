// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexedShooter extends CommandBase {
  /** Creates a new IndexedShooter. */
  public IndexedShooter(int shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSpeed = this.shooterSpeed;
  }
  int shooterSpeed = 0;
  int indexerTime = 1; //TODO: Change these three variables.
  int indexerSpeed = 0;

  TalonFX shooterMotor = new TalonFX(1); //TODO: Change this value..
  TalonFX indexerMotor = new TalonFX(2); //TODO: Change this also...

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //when this command is run the button is being held down
    if(checkIndexer() && spinShooter()){
      runIndexer(indexerTime);
    }
  }

  private boolean spinShooter(){
    int speed = 0; //TODO: Should be equal to whatever the motors reported speed is.

    shooterMotor.set(TalonFXControlMode.PercentOutput, shooterSpeed);

    if(speed >= shooterSpeed){ return true; }
    else{ return false; }
  }

  private boolean checkIndexer(){
    //TODO: do some funny magic here later
    //For now I'm just returning something so it stops yelling at me >:(
    return false;
  }

  private void runIndexer(int t){
    //Timed at some speed
    int localTime=t;
    while(localTime > 0){
      shooterMotor.set(TalonFXControlMode.PercentOutput, indexerSpeed);
    }
    shooterMotor.set(TalonFXControlMode.PercentOutput, 0);
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
