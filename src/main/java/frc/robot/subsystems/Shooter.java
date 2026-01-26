package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Shooter extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter";

    private static final double GEAR_RATIO = -1;
    private static final double WHEEL_DIAMETER_IN_METERS = 0.1016;//converted 4 inches to meters

    private final TalonFXWrapper masterMotor;

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

    private Shooter(String namespaceName, TalonFXWrapper masterMotor, TalonFXWrapper slaveMiddleMotor,
                    TalonFXWrapper slaveLeftMotor) {
        super(namespaceName, masterMotor, slaveMiddleMotor, slaveLeftMotor);
        this.masterMotor = masterMotor;
        slaveMiddleMotor.follow(masterMotor);
        slaveLeftMotor.follow(masterMotor);
        configureDashboard();
        configureRelativeEncoder();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motors speed", motorController::get);
    }

    private void configureRelativeEncoder() {
        masterMotor.setEncoderConversionFactor(GEAR_RATIO * WHEEL_DIAMETER_IN_METERS);
    }
}
