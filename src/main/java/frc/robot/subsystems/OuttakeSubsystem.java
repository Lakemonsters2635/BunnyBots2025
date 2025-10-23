// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  /** Creates a new OuttakeSubsystem. */
  
  TalonFX outTakeMotor;
  
  TalonFX stopOuttakeMotor;

  public OuttakeSubsystem() {
    outTakeMotor = new TalonFX(0);
  }

  public void motorOutTake() {
    outTakeMotor.setVoltage(3);
  }

  public void stopmotorOutTake() {
    stopOuttakeMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
