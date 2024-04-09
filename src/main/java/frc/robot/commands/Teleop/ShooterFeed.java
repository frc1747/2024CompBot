// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;
import frc.robot.subsystems.PivotShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class ShooterFeed extends Command {
  private Feeder feeder;
  private Intake intake;
  private PivotShooter shooterPivot;
  private PivotIntake intakePivot;
  private int flip;
  
  /** Creates a new ShooterFeed. */
  public ShooterFeed(Feeder feeder, Intake intake, PivotShooter shooterPivot, PivotIntake intakePivot, int flip) {
    this.feeder = feeder;
    this.intake = intake;
    this.shooterPivot = shooterPivot;
    this.intakePivot = intakePivot;
    this.flip = flip;
    addRequirements(feeder, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Braden wants this way flip is used to change the shooter feed
    intake.setRollerPower(-Constants.FeederConstants.TRANSITION_SPEED * flip);
    feeder.setShooterFeedPower(Constants.FeederConstants.TRANSITION_SPEED * flip);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(shooterPivot.getPosition() <= 250 || shooterPivot.switchPressed())) {
      if (!(intakePivot.getPosition() <= 250 || intakePivot.switchPressed())) {
        intake.setRollerPower(0.0);
        return;
      }
    }
    intake.setRollerPower(-Constants.FeederConstants.TRANSITION_SPEED * flip);
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
