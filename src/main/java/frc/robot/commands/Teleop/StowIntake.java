// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;

public class StowIntake extends Command {
  PivotIntake intakePivot;
  Intake intake;
  
  public StowIntake(Intake intake, PivotIntake intakePivot) {
    this.intakePivot = intakePivot;
    this.intake = intake;
    addRequirements(intake, intakePivot);
  }

  @Override
  public void initialize() {
    intake.setRollerPower(Constants.IntakeConstants.ROLLER_SPEED_ADJUST_NOTE);
    intakePivot.setHingePower(-Constants.IntakeConstants.PIVOT_IN_SPEED);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intakePivot.setHingePower(0.0);
    intake.setRollerPower(0.0);
  }

  @Override
  public boolean isFinished() {
    // System.out.println("switch pressed: " + intakePivot.switchPressed());
    return intakePivot.switchPressed();
    // return (intakePivot.getPosition() <= Constants.IntakeConstants.STOWED);
  }
}
