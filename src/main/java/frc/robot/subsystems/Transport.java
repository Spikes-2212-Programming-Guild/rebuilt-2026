package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Transport extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "transport";

    private static Transport instance;

    private final TalonFXWrapper talonFX;

    protected Transport(String namespaceName, TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
    }

    public static Transport getInstance() {
        if (instance == null) {
            instance = new Transport(NAMESPACE_NAME, new TalonFXWrapper(RobotMap.CAN.TRANSPORT_TALON_FX_ID));
        }
        return instance;
    }

    protected void apply(double speed) {
        super.apply(speed);
    }

    public void stop() {
        super.stop();
    }
}