package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Kicker extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "transport";

    public static final double SPEED = -1;
    private static final int SMART_CURRENT_LIMIT = 40;
    private static final int SECONDARY_CURRENT_LIMIT = 40;

    private final SparkWrapper sparkMax;

    private static Kicker instance;

    public static Kicker getInstance() {
        if (instance == null) {
            instance = new Kicker(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.TRANSPORT_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    public Kicker(String namespaceName, SparkWrapper sparkMax) {
        super(namespaceName, sparkMax);
        this.sparkMax = sparkMax;
        configureMotor();
        configureDashboard();
    }

    private void configureMotor() {
        sparkMax.restoreFactoryDefaults();
        sparkMax.setIdleMode(SparkBaseConfig.IdleMode.kCoast);
        sparkMax.setInverted(false);
        sparkMax.getSparkConfiguration().apply(sparkMax.getSparkConfiguration()
                        .smartCurrentLimit(SMART_CURRENT_LIMIT)
//                .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
        );
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("velocity", sparkMax::getVelocity);
    }
}
