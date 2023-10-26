// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.blower;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.control.motors.NKTalonFX;

public class Blower extends SubsystemBase {
  /** Creates a new blower. */
  NKTalonFX blowerMotor;
  public Blower(){
    blowerMotor = new NKTalonFX(7);
  }

  public void Start() {
    blowerMotor.set(0.2);
  }

  public void Stop(){
    blowerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
