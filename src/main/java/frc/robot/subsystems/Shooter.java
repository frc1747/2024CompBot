package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.ejml.ops.FConvertArrays;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX shooting;
  private int id;

  /** Creates a new Shooter. */
  public Shooter(int id) {
    this.id = id ;
    shooting = new TalonFX(id);
    shooting.setNeutralMode(NeutralMode.Brake);
  }

  public void setShooterPower(double power) {
    shooting.set(ControlMode.PercentOutput, power);
  }
  public double getPosition() {
    return shooting.getSelectedSensorPosition();
  }

  public double getSpeed() {
    return shooting.getSelectedSensorVelocity();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", getSpeed());

    if (getSpeed() < Constants.ShooterConstants.FLYWHEEL_HIGH_SPEED) {
      SmartDashboard.putBoolean("Shooter Ready?", true);
    } else {
      SmartDashboard.putBoolean("Shooter Ready?", false);
    }
  }
}
