package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap;

public class Shooter extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter";
    private static final String CANIVORE_NAME = "canivore";

    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_IN_METERS = Units.inchesToMeters(4);
    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = -1;

    private final TalonFXWrapper leftTalonFX;
    private final TalonFXWrapper middleTalonFX;
    private final TalonFXWrapper rightTalonFX;

    private static final boolean LEFT_MOTOR_INVERTED = true;
    private static final boolean MIDDLE_MOTOR_INVERTED = true;
    private static final boolean RIGHT_MOTOR_INVERTED = true;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_UPPER_TALON_FX_ID, new CANBus(CANIVORE_NAME)),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_MIDDLE_TALON_FX_ID, new CANBus(CANIVORE_NAME)),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_LOWER_TALON_FX_ID, new CANBus(CANIVORE_NAME)));
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
        leftTalonFX.restoreFactoryDefaults();
        middleTalonFX.restoreFactoryDefaults();
        rightTalonFX.restoreFactoryDefaults();

        leftTalonFX.setIdleMode(NeutralModeValue.Coast);
        middleTalonFX.setIdleMode(NeutralModeValue.Coast);
        rightTalonFX.setIdleMode(NeutralModeValue.Coast);

        middleTalonFX.follow(leftTalonFX);
        rightTalonFX.follow(leftTalonFX);

        leftTalonFX.setInverted(RIGHT_MOTOR_INVERTED);
        middleTalonFX.setInverted(MIDDLE_MOTOR_INVERTED);
        rightTalonFX.setInverted(LEFT_MOTOR_INVERTED);

        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT);
//                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        leftTalonFX.getConfigurator().apply(limitsConfigs);
        middleTalonFX.getConfigurator().apply(limitsConfigs);
        rightTalonFX.getConfigurator().apply(limitsConfigs);
    }

    private void configureRelativeEncoder() {
        leftTalonFX.setEncoderConversionFactor(GEAR_RATIO * WHEEL_DIAMETER_IN_METERS);
        middleTalonFX.setEncoderConversionFactor(GEAR_RATIO * WHEEL_DIAMETER_IN_METERS);
        rightTalonFX.setEncoderConversionFactor(GEAR_RATIO * WHEEL_DIAMETER_IN_METERS);
    }

    public double getVelocity() {
        return leftTalonFX.getVelocity();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("left velocity", leftTalonFX::getVelocity);
        namespace.putNumber("middle velocity", middleTalonFX::getVelocity);
        namespace.putNumber("right velocity", rightTalonFX::getVelocity);
    }
}
