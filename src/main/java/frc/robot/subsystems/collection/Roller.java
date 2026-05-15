package frc.robot.subsystems.collection;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Roller extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "roller";

    private final TalonFXWrapper talonFX;

    public static final double SPEED = 1;

    private static final double CURRENT_LIMIT_AMP = 40.0;

    private static Roller instance;

    public static Roller getInstance() {
        if (instance == null) {
            instance = new Roller(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_TALON_FX_ID,
                            new CANBus("canivore")));
        }
        return instance;
    }

    private Roller(String namespaceName, TalonFXWrapper sparkMax) {
        super(namespaceName, sparkMax);
        this.talonFX = sparkMax;
        talonFX.restoreFactoryDefaults();
        setCurrentLimit(CURRENT_LIMIT_AMP);
        configureDashboard();
    }

    public void setCurrentLimit(double limit) {
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(limit));
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motor velocity", talonFX::getVelocity);
    }
}
