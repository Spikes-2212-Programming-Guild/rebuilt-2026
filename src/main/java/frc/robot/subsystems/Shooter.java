package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Shooter extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter";

    private static Shooter instance;

    public static Shooter getInstance() {
        if(instance == null){
            instance = new Shooter(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_UPPER_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_MIDDLE_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_LOWER_TALON_FX_ID));
        }
        return instance;
    }

    private Shooter(String namespaceName, TalonFXWrapper rightMotor, TalonFXWrapper middleMotor,
                    TalonFXWrapper leftMotor) {
        super(namespaceName, rightMotor, middleMotor, leftMotor);
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("shooter motors", super.motorController::get);
    }
}
