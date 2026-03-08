package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SpinningMagazine extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "spinning magazine";

    private final TalonFXWrapper talonFX;

    public static final double SPEED = 0.3;

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(NAMESPACE_NAME, new TalonFXWrapper(
                    RobotMap.CAN.SPINNING_MAGAZINE_TALON_FX_ID, new CANBus("canivore")));
        }
        return instance;
    }

    private SpinningMagazine(String namespaceName, TalonFXWrapper sparkMax) {
        super(namespaceName, sparkMax);
        this.talonFX = sparkMax;
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("sparkMax speed", talonFX::getVelocity);
    }
}
