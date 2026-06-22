package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class CollectionMovementItay extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "collection movement itay";

    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = -1;
    private static final double STALL_CURRENT = -1;

    private static final double UP_POSE = 0;
    private static final double DOWN_POSE = -1;

    private final TalonFXWrapper talonFX;

    private static CollectionMovementItay instance;

    public static CollectionMovementItay getInstance() {
        if (instance == null) {
            instance = new CollectionMovementItay(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID, new CANBus("canivore")));
        }
        return instance;
    }

    private CollectionMovementItay(String namespaceName, TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        configureMotor();
        configureDashboard();
    }

    private void configureMotor() {
        talonFX.restoreFactoryDefaults();
        talonFX.setInverted(false);
        talonFX.setIdleMode(NeutralModeValue.Brake);
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
//                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
        );
    }

    @Override
    public boolean canMove(double speed) {

        boolean movingDown = speed < 0;
        boolean stall = talonFX.getCurrent() > STALL_CURRENT;
        double position = talonFX.getPosition();

//        if (stall) {
//            if (movingDown) {
//                talonFX.setPosition(DOWN_POSE);
//            }

//            return false;
//        }

//        if (movingDown && position < DOWN_POSE) {
//            return false;
//        }
//
//        if (!movingDown && position > UP_POSE) {
//            return false;
//        }

        return true;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("current", talonFX::getCurrent);
        namespace.putNumber("velocity", talonFX::getVelocity);
        namespace.putNumber("relative pos", talonFX::getPosition);
    }
}
