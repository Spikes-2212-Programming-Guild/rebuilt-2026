package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Transport extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "transport";

    private static Transport instance;

    private final TalonFXWrapper motor;


    public Transport(String namespaceName, TalonFXWrapper motor) {
        super(namespaceName, motor);
        this.motor = motor;


    }

    public static Transport getInstance() {
        if (instance == null) {
            instance = new Transport(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.TRANSPORT_TALON_FX_ID));
        }
        return instance;
    }
}
