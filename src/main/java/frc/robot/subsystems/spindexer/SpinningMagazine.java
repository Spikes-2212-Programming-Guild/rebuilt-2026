package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SpinningMagazine extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "spinning magazine";

    private final TalonFXWrapper talonFX;

    public static final double SPEED = 1;
    private static final double SUPPLY_CURRENT_LIMIT = 40;
//    private static final double STATOR_CURRENT_LIMIT = -1;

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(NAMESPACE_NAME, new TalonFXWrapper(
                    RobotMap.CAN.SPINNING_MAGAZINE_TALON_FX_ID, new CANBus("canivore")));
        }
        return instance;
    }

    private SpinningMagazine(String namespaceName, TalonFXWrapper talonFx) {
        super(namespaceName, talonFx);
        this.talonFX = talonFx;
        configureMotor();
        configureDashboard();
    }

    private void configureMotor() {
        talonFX.restoreFactoryDefaults();
        talonFX.setIdleMode(NeutralModeValue.Coast);
        talonFX.setInverted(false);
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
//                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
        );
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("velocity", talonFX::getVelocity);
    }
}
