package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;

public class CollectionMovement extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "collection movement";
    private static final double DEGREES_IN_ROTATION = 360;
    private static final double CURRENT_LIMIT_AMP = 40;
    private static final double OPEN_POSE = -1;
    private static final double CLOSE_POSE = -1;
    private final DutyCycleEncoder absoluteEncoder;
    private final TalonFXWrapper talonFX;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new DutyCycleEncoder(RobotMap.DIO.COLLECTION_MOVEMENT_THROUGH_BORE_ID),
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DutyCycleEncoder absoluteEncoder,
                               TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        this.absoluteEncoder = absoluteEncoder;
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMP));
        configureDashboard();
    }

    public double getAbsDegrees() {
        return absoluteEncoder.get() * DEGREES_IN_ROTATION;
    }

    public void resetRelativeEncoder() {
        talonFX.setPosition(getAbsDegrees());
    }

    @Override
    public boolean canMove(double speed) {
        return ( getAbsDegrees() > OPEN_POSE && speed < 0) ||
                (getAbsDegrees() < CLOSE_POSE && speed > 0);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("velocity", talonFX::getVelocity);
        namespace.putNumber("relative", talonFX::getPosition);
        namespace.putNumber("absolute", this::getAbsDegrees);
    }
}
