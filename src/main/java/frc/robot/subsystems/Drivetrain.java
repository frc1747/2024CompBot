// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public Pigeon2 gyro;

    public Drivetrain() {
        // Create an instance of the gyro, config its parameters, and zero it out, 
        // making whichever direction the robot is facing when robot code is initialized 0
        gyro = new Pigeon2(Constants.DrivetrainConstants.PIGEON_ID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        // Define all swerve modules with the constants defined in Constants.java
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.DrivetrainConstants.FRONT_LEFT.constants),
            new SwerveModule(1, Constants.DrivetrainConstants.FRONT_RIGHT.constants),
            new SwerveModule(2, Constants.DrivetrainConstants.REAR_LEFT.constants),
            new SwerveModule(3, Constants.DrivetrainConstants.REAR_RIGHT.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.DrivetrainConstants.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw()
            )
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DrivetrainConstants.MAX_SPEED);

        for(SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public Rotation2d getYaw() {
        return (Constants.DrivetrainConstants.invertGyro) ? Rotation2d.fromDegrees(180 - gyro.getYaw().getValue()) : Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        // Update the odometry every robot cycle tick
        swerveOdometry.update(getYaw(), getModulePositions());
        SmartDashboard.putString("Robot Location: ", getPose().getTranslation().toString());
        SmartDashboard.putString("Yaw status", getYaw().toString());

        for (SwerveModule mod : swerveModules) {
            int printModule = mod.moduleNumber + 1;
            SmartDashboard.putNumber("Mod " + printModule + " Cancoder", mod.getCancoder().getDegrees());
            SmartDashboard.putNumber("Mod " + printModule + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + printModule + " Velocity", mod.getState().speedMetersPerSecond); 
            SmartDashboard.putNumber("Mod " + printModule + " Position", mod.getPosition().distanceMeters);  
        }
    }

    public class SwerveModule {
        public int moduleNumber;

        private Rotation2d angleOffset;
        private Rotation2d lastAngle;

        private CANSparkMax angleMotor;
        private CANSparkMax driveMotor;

        private RelativeEncoder driveEncoder;
        private RelativeEncoder integratedAngleEncoder;
        private CANcoder angleEncoder;

        private SparkPIDController driveController;
        private SparkPIDController angleController;

        private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DrivetrainConstants.DRIVE_KS, Constants.DrivetrainConstants.DRIVE_KV, Constants.DrivetrainConstants.DRIVE_KA);

        public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
            this.moduleNumber = moduleNumber;
            this.angleOffset = moduleConstants.angleOffset;

            // CANCoder
            angleEncoder = new CANcoder(moduleConstants.cancoderID);
            configAngleEncoder();

            // Angle Motor
            angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
            integratedAngleEncoder = angleMotor.getEncoder();
            angleController = angleMotor.getPIDController();
            configAngleMotor();

            // Drive Motor
            driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
            driveEncoder = driveMotor.getEncoder();
            driveController = driveMotor.getPIDController();
            configDriveMotor();

            lastAngle = getState().angle;
        }

        private void configAngleEncoder() {
            angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
            CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
            angleEncoder.getConfigurator().apply(RobotContainer.ctreConfigs.swerveCanCoderConfig);
        }

        private void configAngleMotor() {
            angleMotor.restoreFactoryDefaults();
            CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
            angleMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.angleContinuousCurrentLimit);
            angleMotor.setInverted(Constants.DrivetrainConstants.angleMotorInvert);
            angleMotor.setIdleMode(Constants.DrivetrainConstants.angleNeutralMode);
            integratedAngleEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.angleConversionFactor);
            angleController.setP(Constants.DrivetrainConstants.ANGLE_KP);
            angleController.setI(Constants.DrivetrainConstants.ANGLE_KI);
            angleController.setD(Constants.DrivetrainConstants.ANGLE_KD);
            angleController.setFF(Constants.DrivetrainConstants.ANGLE_KF);
            angleMotor.enableVoltageCompensation(Constants.DrivetrainConstants.voltageComp);
            angleMotor.burnFlash();
            resetToAbsolute();
        }

        private void configDriveMotor() {
            driveMotor.restoreFactoryDefaults();
            CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
            driveMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.driveContinuousCurrentLimit);
            driveMotor.setInverted(Constants.DrivetrainConstants.driveMotorInvert);
            driveMotor.setIdleMode(Constants.DrivetrainConstants.driveNeutralMode);
            driveEncoder.setVelocityConversionFactor(Constants.DrivetrainConstants.driveConversionVelocityFactor);
            driveEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.driveConversionPositionFactor);
            driveController.setP(Constants.DrivetrainConstants.DRIVE_KP);
            driveController.setI(Constants.DrivetrainConstants.DRIVE_KI);
            driveController.setD(Constants.DrivetrainConstants.DRIVE_KD);
            driveController.setFF(Constants.DrivetrainConstants.DRIVE_KF);
            driveMotor.enableVoltageCompensation(Constants.DrivetrainConstants.voltageComp);
            driveMotor.burnFlash();
            driveEncoder.setPosition(0.0);
        }
        
        public void resetToAbsolute() {
            double absolutePosition = getCancoder().getDegrees() - angleOffset.getDegrees();
            angleMotor.getEncoder().setPosition(absolutePosition);
        }

        public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
            desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
            setAngle(desiredState);
            setSpeed(desiredState, isOpenLoop);
        }

        public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
            if (isOpenLoop) {
                double percentOutput = desiredState.speedMetersPerSecond / Constants.DrivetrainConstants.MAX_SPEED;
                driveMotor.set(percentOutput);
            } else {
                driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond)
                );
            }
        }

        public void setAngle(SwerveModuleState desiredState) {
            // Prevent roating module if speed is less than 1%
            Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DrivetrainConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle;
            angleController.setReference(angle.getDegrees(), ControlType.kPosition);
            lastAngle = angle;
        }

        public Rotation2d getAngle() {
            return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        }

        public Rotation2d getCancoder() {
            return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
        }
    }

    public class OnboardModuleState {
        /**
         * Minimize the change in heading the desired swerve module state would require by potentially
         * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
         * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
         * support for continuous input.
         *
         * @param desiredState The desired state.
         * @param currentAngle The current module angle.
         */
        public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
            double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
            double targetSpeed = desiredState.speedMetersPerSecond;
            double delta = targetAngle - currentAngle.getDegrees();
            if (Math.abs(delta) > 90) {
                targetSpeed = -targetSpeed;
                targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
            }
            return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
        }

        /**
         * @param scopeReference Current Angle
         * @param newAngle Target Angle
         * @return Closest angle within scope
         */
        private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
            double lowerBound;
            double upperBound;
            double lowerOffset = scopeReference % 360;
            if (lowerOffset >= 0) {
                lowerBound = scopeReference - lowerOffset;
                upperBound = scopeReference + (360 - lowerOffset);
            } else {
                upperBound = scopeReference - lowerOffset;
                lowerBound = scopeReference - (360 + lowerOffset);
            }
            while (newAngle < lowerBound) {
                newAngle += 360;
            }
            while (newAngle > upperBound) {
                newAngle -= 360;
            }
            if (newAngle - scopeReference > 180) {
                newAngle -= 360;
            } else if (newAngle - scopeReference < -180) {
                newAngle += 360;
            }
            return newAngle;
        }
    }
}
