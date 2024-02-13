package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public class CTREConfigs {
    public MagnetSensorConfigs swerveCanCoderConfig;

    public CTREConfigs() {
        swerveCanCoderConfig = new MagnetSensorConfigs();

        // Swerve CANCoder Configuration
        swerveCanCoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.SensorDirection = Constants.DrivetrainConstants.canCoderInvert;
    }
}    
