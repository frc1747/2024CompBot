// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;
import frc.robot.subsystems.PivotShooter;

public class CleanIntake extends Command {
  private final PivotIntake intakePivot;
  private final Intake intake;
  private boolean isReversed;
  
  /** Creates a new DefenseBotShooterPreset. */
  public CleanIntake(PivotIntake intakePivot, Intake intake) {
    this.intakePivot = intakePivot;
    this.intake = intake;
    this.isReversed = false;
    addRequirements(intakePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakePivot.goToClean();
    intake.setRollerPower(-Constants.IntakeConstants.ROLLER_SPEED_CLEAN);
  }

  // can be improved tolerance with PID
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.setHingePower(0.0);
    intake.setRollerPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
