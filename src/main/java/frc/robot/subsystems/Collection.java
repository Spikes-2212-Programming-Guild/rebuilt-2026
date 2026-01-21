package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Collection extends MotoredGenericSubsystem {

    private final static String namespaceName="Collection";

    public Collection(String namespaceName, SmartMotorController motorController) {
        super(namespaceName, motorController);
    }

    private static Collection instance;

    public static Collection getInstance() {
        if (instance == null) {
            instance = new Collection(namespaceName,
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
