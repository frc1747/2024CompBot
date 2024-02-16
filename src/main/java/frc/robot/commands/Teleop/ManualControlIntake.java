// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ManualControlIntake extends Command {
  private Intake intake;
  private Joystick controller;

  /** Creates a new ManualControl. */
  public ManualControlIntake(Intake intake, Joystick controller) {
    this.intake = intake;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final int leftY = XboxController.Axis.kLeftY.value;
    final int rightY = XboxController.Axis.kRightY.value;
    intake.setRollerPower(deadzone(controller.getRawAxis(rightY)));
    double left = deadzone(controller.getRawAxis(leftY));
    if (left == 0) {
      intake.setHingePower(0.0);
    } else if (left < 0 && intake.getPosition() > Constants.IntakeConstants.DOWN_LIMIT) {
      intake.setHingePower(left);
    } else if (left > 0 && intake.getPosition() < Constants.IntakeConstants.UP_LIMIT) {
      intake.setHingePower(left);
    }
  }

  private double deadzone(double power) {
    double returned = 0.0;
    if (power > 0.10) {
      returned = (power - 0.10) / 0.90;
    } else if (power < -0.10) {
      returned = (power + 0.10) / 0.90;
    }
    return returned;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
