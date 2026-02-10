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

    private static final boolean MASTER_INVERTED = false;
    private static final boolean SLAVE_MIDDLE_INVERTED = false;
    private static final boolean SLAVE_LEFT_INVERTED = false;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_UPPER_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_MIDDLE_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_LOWER_TALON_FX_ID));
        }
        return instance;
    }

    private Shooter(String namespaceName, TalonFXWrapper rightTalonFX, TalonFXWrapper middleTalonFX,
                    TalonFXWrapper leftTalonFX) {
        super(namespaceName, rightTalonFX);
        this.masterTalonFX = rightTalonFX;
        middleTalonFX.follow(rightTalonFX);
        leftTalonFX.follow(rightTalonFX);
        rightTalonFX.setInverted(MASTER_INVERTED);
        middleTalonFX.setInverted(SLAVE_MIDDLE_INVERTED);
        leftTalonFX.setInverted(SLAVE_LEFT_INVERTED);
        rightTalonFX.getConfigurator().apply(new CurrentLimitsConfigs().
                withSupplyCurrentLimit(CURRENT_LIMIT_AMP));
        configureRelativeEncoder();
        configureDashboard();
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
