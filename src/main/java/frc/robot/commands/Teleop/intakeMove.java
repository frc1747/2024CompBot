// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;

public class intakeMove extends Command {
  private PivotIntake intake;
  public boolean done = false;
  double pow;
  /** Creates a new IntakeUp. */
  public intakeMove(PivotIntake intake, DoubleSupplier power ) {
    this.intake = intake;
    this.pow = power.getAsDouble();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //intake.liftIntake();
   // done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    intake.setHingePower(pow * 0.25);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
