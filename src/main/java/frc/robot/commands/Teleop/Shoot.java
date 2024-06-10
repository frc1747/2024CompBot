// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  private Shooter LShooter;
  private Shooter RShooter;

  int flip;

  /** Creates a new SHoot. */
  public Shoot(Shooter Lshooter,Shooter RShooter, int flip) {
    this.LShooter = Lshooter;
    this.RShooter = RShooter;
    this.flip = flip;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Lshooter,RShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LShooter.setShooterPower(Constants.ShooterConstants.SHOOT_SPEED * flip);  
    RShooter.setShooterPower(Constants.ShooterConstants.SHOOT_SPEED * -flip); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LShooter.setShooterPower(0.0);
    RShooter.setShooterPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
