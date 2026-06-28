package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "collection movement";

    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 30;

    private static final double MOTION_EPSILON = -1.0;     // Minimum degrees change to be considered "moving"
    private static final double STALL_TIME_LIMIT = -1.0;   // Seconds to wait before triggering stall protection

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
        configureMotor();
        configureDashboard();
    }

    private void configureMotor() {
        talonFX.restoreFactoryDefaults();
        talonFX.setInverted(false);
        talonFX.setIdleMode(NeutralModeValue.Brake);
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
        );
        talonFX.resetPosition();
    }


    public double getAbsDegrees() {
        return absoluteEncoder.get();
    }

    private void checkForStall() {
        double currentPos = getAbsDegrees();
        double currentTime = Timer.getFPGATimestamp();

        double deltaPos = Math.abs(currentPos - lastPositionDegrees);

        if (deltaPos > MOTION_EPSILON) {
            lastMoveTime = currentTime;
            lastPositionDegrees = currentPos;
            isStalled = false;
        }

        else {
            if (currentTime - lastMoveTime > STALL_TIME_LIMIT) {
                isStalled = true;
            }
        }
    }
    @Override
    public boolean canMove(double speed) {
        return true;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("current", talonFX::getCurrent);
        namespace.putNumber("velocity", talonFX::getVelocity);
        namespace.putRunnable("reset pos", talonFX::resetPosition);
        namespace.putNumber("relative pos", talonFX::getPosition);
    }

    public void setCoast() {
        talonFX.setIdleMode(NeutralModeValue.Coast);
    }
}
