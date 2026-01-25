package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Collection extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "collection";

    private final TalonFXWrapper motor;
    private static Collection instance;

    public static Collection getInstance() {
        if (instance == null) {
            instance = new Collection(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_TALON_FX_ID));
        }
        return instance;
    }

    private Collection(String namespaceName, TalonFXWrapper motor) {
        super(namespaceName, motor);
        this.motor = motor;
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motor velocity", motor::getVelocity);
    }
}
