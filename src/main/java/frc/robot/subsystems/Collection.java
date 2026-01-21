package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Collection extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME ="collection";

    public Collection(String NAMESPACE_NAME, SmartMotorController motorController) {
        super(NAMESPACE_NAME, motorController);
    }

    private static Collection instance;

    public static Collection getInstance() {
        if (instance == null) {
            instance = new Collection(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_TALON_FX_ID));
        }
        return instance;
    }

    // will change in future when it will be decided on what causes it to start/stop
    @Override
    public boolean canMove(double speed) {
        return super.canMove(speed);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("speed", motorController::get);
    }
}
