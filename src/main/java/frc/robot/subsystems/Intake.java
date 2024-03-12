// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax rollerOne;
  private CANSparkMax rollerTwo;
  private DigitalInput limitSwitch;

  /** Creates a new Intake. */
  public Intake() {
    rollerOne = new CANSparkMax(Constants.IntakeConstants.ROLLER_ONE, MotorType.kBrushless);
    rollerTwo = new CANSparkMax(Constants.IntakeConstants.ROLLER_TWO, MotorType.kBrushless);
    limitSwitch = new DigitalInput(Constants.IntakeConstants.NOTE_LIMIT_SWITCH);
  }

  public void setRollerPower(double power) {
    rollerOne.set(power);
    rollerTwo.set(power);
  }

  public boolean switchPressed() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
