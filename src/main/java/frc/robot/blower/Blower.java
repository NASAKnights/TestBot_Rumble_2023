// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.blower;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.control.motors.NKTalonFX;
import frc.lib.control.motors.NKVictorSPX;

public class Blower extends SubsystemBase {
  /** Creates a new blower. */
  NKVictorSPX blowerMotor;
  public Blower(){
    blowerMotor = new NKVictorSPX(8);
    blowerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void start() {
    blowerMotor.set(0.6);
  }

  public void stop(){
    blowerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
