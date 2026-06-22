package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "collection movement";
    private static final double CURRENT_LIMIT_AMP = 40;

    private final TalonFXWrapper talonFX;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID, new CANBus("canivore")));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMP));
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
//        return (talonFX.getPosition() > OPEN_POSE && speed < 0) || (talonFX.getPosition() < CLOSE_POSE && speed > 0);
        return true;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("velocity", talonFX::getVelocity);
        namespace.putNumber("relative pos", talonFX::getPosition);
    }
}
