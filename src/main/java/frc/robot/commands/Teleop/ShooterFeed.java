// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShooterFeed extends Command {
  private Feeder feeder;
  private Intake intake;
  
  /** Creates a new ShooterFeed. */
  public ShooterFeed(Feeder feeder, Intake intake) {
    this.feeder = feeder;
    this.intake = intake;
    addRequirements(feeder, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRollerPower(-0.5);
    feeder.setShooterFeedPower(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerPower(0.0);
    feeder.setShooterFeedPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
