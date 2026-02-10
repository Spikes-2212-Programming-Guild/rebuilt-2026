package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Shooter extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter";

    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_IN_METERS = 0.1016;//converted 4 inches to meters
    private static final double CURRENT_LIMIT_AMP = 40;

    private final TalonFXWrapper leftTalonFX;
    private final TalonFXWrapper middleTalonFX;
    private final TalonFXWrapper rightTalonFX;

    private static final boolean LEFT_MOTOR_INVERTED = false;
    private static final boolean MIDDLE_MOTOR_INVERTED = false;
    private static final boolean RIGHT_MOTOR_INVERTED = false;

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

    private Shooter(String namespaceName, TalonFXWrapper leftTalonFX, TalonFXWrapper middleTalonFX,
                    TalonFXWrapper rightTalonFX) {
        super(namespaceName, leftTalonFX);
        this.leftTalonFX = leftTalonFX;
        this.middleTalonFX = middleTalonFX;
        this.rightTalonFX = rightTalonFX;
        configureMotors();
        configureRelativeEncoder();
        configureDashboard();
    }

    private void configureMotors() {
        middleTalonFX.follow(leftTalonFX);
        rightTalonFX.follow(leftTalonFX);
        leftTalonFX.setInverted(RIGHT_MOTOR_INVERTED);
        middleTalonFX.setInverted(MIDDLE_MOTOR_INVERTED);
        rightTalonFX.setInverted(LEFT_MOTOR_INVERTED);
        leftTalonFX.getConfigurator().apply(new CurrentLimitsConfigs().
                withSupplyCurrentLimit(CURRENT_LIMIT_AMP));
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motors speed", motorController::get);
    }

    private void configureRelativeEncoder() {
        leftTalonFX.setEncoderConversionFactor(GEAR_RATIO * WHEEL_DIAMETER_IN_METERS);
    }

    public void setSpeed(double speed) {
        leftTalonFX.set(speed);
    }

    public double getVelocity() {
        return leftTalonFX.getVelocity();
    }
}
