// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autoscommands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends InstantCommand {
  /** Creates a new Intake. */
  Shooter lshoot;
  Shooter rshoot;
  Intake pIntake;
  Feeder pfeeder;
  boolean done = false;

  
  String type;
  public ShootAuto(Shooter Sleft, Shooter Sright,Intake pIntake2,  Feeder feedelliot , String type  ) {

  
    this.rshoot = Sright;
    this.lshoot = Sleft;
    this.pIntake = pIntake2;
    this.type = type;
    this.pfeeder = feedelliot;




    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (type.equals("shoot" )) {
      rshoot.setShooterPower(-.80); 
      lshoot.setShooterPower(.80);
      Timer.delay(.5);
      pIntake.setRollerPower(-.8);
      pfeeder.setShooterFeedPower(1);
      try {
        TimeUnit.SECONDS.sleep(1);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      rshoot.setShooterPower(0);
      lshoot.setShooterPower(0);
      pIntake.setRollerPower(0);
      pfeeder.setShooterFeedPower(0);
    }
    done = true;

    end(done);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rshoot.setShooterPower(0);
    lshoot.setShooterPower(0); 
    pIntake.setRollerPower(0);
    pfeeder.setShooterFeedPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
