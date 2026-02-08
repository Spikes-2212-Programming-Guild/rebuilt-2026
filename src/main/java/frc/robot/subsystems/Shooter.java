package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Shooter extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter";

    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_IN_METERS = 0.1016;//converted 4 inches to meters
    private final double CURRENT_LIMIT_AMP = 40;

    private final TalonFXWrapper masterTalonFX;

    private final boolean IS_INVERTED;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_UPPER_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_MIDDLE_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_LOWER_TALON_FX_ID),
                    getInstance().IS_INVERTED);
        }
        return instance;
    }

    private Shooter(String namespaceName, TalonFXWrapper masterTalonFX, TalonFXWrapper slaveMiddleTalonFX,
                    TalonFXWrapper slaveLeftTalonFX, boolean isInverted) {
        super(namespaceName, masterTalonFX);
        slaveMiddleTalonFX.follow(masterTalonFX);
        slaveLeftTalonFX.follow(masterTalonFX);
        this.masterTalonFX = masterTalonFX;
        this.IS_INVERTED = isInverted;
        masterTalonFX.setInverted(isInverted);
        masterTalonFX.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT_AMP));
        configureDashboard();
        configureRelativeEncoder();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motors speed", motorController::get);
    }

    private void configureRelativeEncoder() {
        masterTalonFX.setEncoderConversionFactor(GEAR_RATIO * WHEEL_DIAMETER_IN_METERS);
    }

    public void setSpeed(double speed) {
        masterTalonFX.set(speed);
    }

    public double getVelocity() {
        return masterTalonFX.getVelocity();
    }

    public void stop() {
        masterTalonFX.stopMotor();
    }
}
