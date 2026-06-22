package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Collection extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "collection";

    private final TalonFXWrapper talonFX;

    public static final double SPEED = 1;

    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = -1;

    private static Collection instance;

    public static Collection getInstance() {
        if (instance == null) {
            instance = new Collection(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_TALON_FX_ID,
                            new CANBus("canivore")));
        }
        return instance;
    }

    private Collection(String namespaceName, TalonFXWrapper talonFx) {
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
