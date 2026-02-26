package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SpinningMagazine extends MotoredGenericSubsystem {

    public static final double SPEED = -1.0;

    private static final String NAMESPACE_NAME = "spinning magazine";

    private static final double CURRENT_LIMIT_AMP = 40;

    private final TalonFXWrapper talonFX;

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SPINNING_MAGAZINE_TALON_FX_ID));
        }
        return instance;
    }

    private SpinningMagazine(String namespaceName, TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT_AMP));
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("sparkMax speed", talonFX::getVelocity);
    }
}
