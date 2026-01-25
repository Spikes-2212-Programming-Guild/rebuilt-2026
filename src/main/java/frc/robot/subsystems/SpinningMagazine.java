package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SpinningMagazine extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "spinning magazine";

    private final TalonFXWrapper motor; //@TODO ask mechanics what motor do we use

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SPINNING_MAGAZINE_TALON_FX_ID));
        }
        return instance;
    }

    private SpinningMagazine(String namespaceName, TalonFXWrapper motor) {
        super(namespaceName, motor);
        this.motor = motor;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motor speed", motor::getVelocity);
    }
}
