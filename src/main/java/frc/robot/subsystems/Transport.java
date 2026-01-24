package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Transport extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "transport";

    private static Transport instance;

    private final TalonFXWrapper talonFX;

    public static Transport getInstance() {
        if (instance == null) {
            instance = new Transport(NAMESPACE_NAME, new TalonFXWrapper(RobotMap.CAN.TRANSPORT_TALON_FX_ID));
        }
        return instance;
    }

    public Transport(String namespaceName, TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
    }
}