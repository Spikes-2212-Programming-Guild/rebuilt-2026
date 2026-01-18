package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Climb extends SmartMotorControllerGenericSubsystem {

    private final static String NAMESPACE_NAME = "climb";

    private static Climb instance;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.CLIMB_TALON_FX_LEFT_ID),
                    new TalonFXWrapper(RobotMap.CAN.CLIMB_TALON_FX_RIGHT_ID));
        }
        return instance;
    }

    public Climb(String namespaceName, TalonFXWrapper leftMotor, TalonFXWrapper rightMotor) {
        super(namespaceName, leftMotor);
        rightMotor.follow(leftMotor, false);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("current left velocity", motorController::get);
    }
}
