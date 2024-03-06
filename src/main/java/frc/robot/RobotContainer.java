// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.Teleop.Climb;
import frc.robot.commands.Teleop.FloorPickup;
import frc.robot.commands.Teleop.Intakeshoot;
import frc.robot.commands.Teleop.Shoot;
import frc.robot.commands.Teleop.ShooterAlignAmp;
import frc.robot.commands.Teleop.ShooterDown;
import frc.robot.commands.Teleop.ShooterFeed;
import frc.robot.commands.Teleop.Shooterarm;
import frc.robot.commands.Teleop.StowIntake;
import frc.robot.commands.LockOn;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.Autoscommands.ManualControlIntake;
import frc.robot.commands.Teleop.TeleopSwerve;
import frc.robot.commands.Teleop.Transition;
import frc.robot.commands.Teleop.intakeMove;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;
import frc.robot.subsystems.PivotShooter;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static CTREConfigs ctreConfigs = new CTREConfigs();

  // subsystems
  public final PivotShooter pShooter = new PivotShooter();
  public final Shooter shooter = new Shooter();
  public final PivotIntake pIntake = new PivotIntake();
  public final Intake intake = new Intake();
  public final Drivetrain drivetrain = new Drivetrain();
  public final Climber leftClimber = new Climber(Constants.ClimberConstants.LEFT, "Left");
  public final Climber rightClimber = new Climber(Constants.ClimberConstants.RIGHT, "Right");
  public final Feeder feeder = new Feeder();
  private final Vision camShooter = new Vision("Shooter");
  private final Vision camBack = new Vision("Back");

  // Braden failing to code

  public final double arm_zero = 0.0;

  // Controllers
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  // Alerts
  private final Alert driverDisconnectedAlert = new Alert("Driver controller is disconnected (port " + driver.getPort() + ").", AlertType.WARNING);
  private final Alert operatorDisconnectedAlert = new Alert("Operator controller is disconnected (port " + operator.getPort() + ").", AlertType.WARNING);

  // BooleanSuppliers
  private final BooleanSupplier rightTrigger = () -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > Short.MAX_VALUE - 10;
  private final BooleanSupplier leftTrigger = () -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > Short.MAX_VALUE - 10;
  private final BooleanSupplier rightBumper = () -> operator.getRawAxis(XboxController.Button.kRightBumper.value) == 1;
  private final BooleanSupplier leftBumper = () -> operator.getRawAxis(XboxController.Button.kLeftBumper.value) == 1;
  private final BooleanSupplier toggleManual = () -> operator.getRawAxis(XboxController.Button.kStart.value) == 1; 
  private  BooleanSupplier b_intakeMovement = () -> Math.abs(operator.getRawAxis(XboxController.Axis.kLeftY.value)) > 0;
  private  BooleanSupplier b_intakein_out = () -> Math.abs(operator.getRawAxis(XboxController.Axis.kLeftX.value)) > 0;
  private final BooleanSupplier b_shooterarm = () -> Math.abs(operator.getRawAxis(XboxController.Axis.kRightY.value)) != 0;

  // Double Suplpliers
  private final DoubleSupplier intakeMovement = () -> operator.getRawAxis(XboxController.Axis.kLeftY.value);
  private final DoubleSupplier intakein_out = () -> operator.getRawAxis(XboxController.Axis.kLeftX.value);
  private final DoubleSupplier shooterarm = () -> operator.getRawAxis(XboxController.Axis.kRightY.value);
  private boolean Toggle;

  
  // climber Controls speeds
  double climberspeed = -.2;
 
  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    pShooter.setEncoderPos(0.0);
    pIntake.setEncoderPos(0.0);

    DriverStation.silenceJoystickConnectionWarning(true);
    // Setup Logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    Toggle = false; 
    drivetrain.setDefaultCommand(
      new TeleopSwerve(
        drivetrain, 
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis), 
        () -> -driver.getRawAxis(rotationAxis), 
        () -> robotCentric.getAsBoolean()
      )
    );
    pIntake.setDefaultCommand(
      new intakeMove(pIntake, intakeMovement));
    
    //intake.setDefaultCommand(
    //   new Intakeshoot(intake, intakein_out));

    pShooter.setDefaultCommand(
      new Shooterarm(pShooter, shooterarm , arm_zero ));
    
      // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));
    // used to swicth the climber going up 0or down

    // Shooter
    new JoystickButton(operator, XboxController.Button.kA.value)
     .whileTrue(new Shoot(shooter , 1));
    new JoystickButton(operator, XboxController.Button.kB.value)
     .whileTrue(new Shoot(shooter , -1 ));

    //  new JoystickButton(operator, XboxController.Button.kX.value)
    //   .whileTrue(new ShooterAlignAmp(pShooter));

    new JoystickButton(operator, XboxController.Button.kX.value)
      .whileTrue(new Transition(feeder , 1));
    new JoystickButton(operator, XboxController.Button.kY.value)
      .whileTrue(new Transition(feeder ,-1));

    // new JoystickButton(operator, XboxController.Axis.kRightY.value)
    //   .whileTrue(new Shooterarm(pShooter , shooterarm));
    
    // intake
    // new JoystickButton(operator, XboxController.Axis.kLeftY.value)
    //   .whileTrue(new intakeMove(pIntake , intakeMovement));

    // new JoystickButton(operator, XboxController.Axis.kLeftX.value)
    //   .whileTrue(new Intakeshoot(intake , intakein_out));

    //Toggle Manual W.I.P
    //new JoystickButton(operator, XboxController.Button.kStart.value)
      
    // climber 
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Climb(leftClimber, Constants.ClimberConstants.CLIMBER_SPEED));
    
    new Trigger(() -> (operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0))
      .whileTrue(new Climb(rightClimber, Constants.ClimberConstants.CLIMBER_SPEED));
    
      // climber 
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Climb(leftClimber, -Constants.ClimberConstants.CLIMBER_SPEED));
    
    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
      .whileTrue(new Climb(rightClimber, Constants.ClimberConstants.CLIMBER_SPEED));
    
    new Trigger(() -> (operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0))
      .whileTrue(new Climb(leftClimber, Constants.ClimberConstants.CLIMBER_SPEED));
    
    new Trigger(() -> (operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0))
      .whileTrue(new Climb(rightClimber, -Constants.ClimberConstants.CLIMBER_SPEED));

    // lock on & Gyro Resest
    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
      .whileTrue(new LockOn(drivetrain, camBack, driver));
    
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
      .onTrue(new ResetGyro(drivetrain));

    // this stuff
    new Trigger(() -> (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0))
      .whileTrue(new FloorPickup(intake, pIntake))
      .whileFalse(new StowIntake(intake, pIntake));

    new JoystickButton(operator, XboxController.Button.kLeftStick.value)
      .whileTrue(new ShooterFeed(feeder, intake));
  }


  /*
   * Checks if the controllers are plugged in and updates their respective alerts
   */
  public void checkControllers() {
    boolean driverConnected = DriverStation.isJoystickConnected(driver.getPort());
    boolean operatorConnected = DriverStation.isJoystickConnected(operator.getPort());

    driverDisconnectedAlert.set(!driverConnected);
    operatorDisconnectedAlert.set(!operatorConnected);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; // Autos.exampleAuto(m_exampleSubsystem);
  }
}
