// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotShooter;
import frc.robot.subsystems.Shooter;

public class Shooterarm extends Command {
  private PivotShooter shooter;
  private DoubleSupplier pow;

  /** Creates a new ShooterAlignAmp. */
  public Shooterarm(PivotShooter shooter, DoubleSupplier pow) {
    this.shooter = shooter;
    this.pow = pow;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //shooter.alignShooterAmp();
    // done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setHingePower(pow.getAsDouble() * .25);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHingePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
