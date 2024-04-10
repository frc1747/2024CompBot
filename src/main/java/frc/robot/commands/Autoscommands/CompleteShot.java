// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autoscommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CompleteShot extends Command {
  Shooter shooter;
  Feeder feeder;
  Intake intake;
  long startTime;
  
  /** Creates a new CompleteShot. */
  public CompleteShot(Shooter shooter, Feeder feeder, Intake intake) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.intake = intake;
    this.startTime = 0;
    addRequirements(shooter, feeder, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
    intake.setRollerPower(-Constants.FeederConstants.TRANSITION_SPEED);
    feeder.setShooterFeedPower(Constants.FeederConstants.TRANSITION_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= 500;
  }
}
