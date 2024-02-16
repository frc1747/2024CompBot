package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX shooting;
  private TalonFX transitioning;
  private TalonFX hinge;

  /** Creates a new Shooter. */
  public Shooter() {
    shooting = new TalonFX(Constants.ShooterConstants.FRONT);
    transitioning = new TalonFX(Constants.ShooterConstants.BACK);
    hinge = new TalonFX(Constants.ShooterConstants.HINGE);
    shooting.setNeutralMode(NeutralMode.Brake);
    transitioning.setNeutralMode(NeutralMode.Brake);

    configPID();
  }

  public void configPID() {
    double[] pidf = new double[] {0.4, 0, 0, 0};
    hinge.config_kP(0, pidf[0]);
    hinge.config_kI(0, pidf[1]);
    hinge.config_kD(0, pidf[2]);
    hinge.config_kF(0, pidf[3]);
  }

  public void setShooterPower(double power) {
    shooting.set(ControlMode.PercentOutput, power);
  }

  public void setTransitionPower(double power) {
    transitioning.set(ControlMode.PercentOutput, power);
  }

  public void setHingePower(double power) {
    hinge.set(ControlMode.PercentOutput, power);
  }

  public void dropShooter() {
    hinge.set(ControlMode.Position, Constants.ShooterConstants.STOWED);
  }

  public void alignShooterSpeaker() {
    hinge.set(ControlMode.Position, 10.0);
  }

  public void alignShooterAmp() {
    hinge.set(ControlMode.Position, Constants.ShooterConstants.AMP);
  }

  public void resetEncoder() {
    hinge.setSelectedSensorPosition(0.0);
  }

  public double getPosition() {
    return hinge.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
