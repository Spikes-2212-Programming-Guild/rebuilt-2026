package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Collection extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "collection";

    private final SparkWrapper sparkMax;

    public static final double SPEED = -1.0;

    private static final double CURRENT_LIMIT_AMP = 40.0;

    private static Collection instance;

    public static Collection getInstance() {
        if (instance == null) {
            instance = new Collection(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.COLLECTION_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private Collection(String namespaceName, SparkWrapper sparkMax) {
        super(namespaceName, sparkMax);
        this.sparkMax = sparkMax;
        setCurrentLimit(CURRENT_LIMIT_AMP);
        configureDashboard();
    }

    public void setCurrentLimit(double limit) {
        sparkMax.getSparkConfiguration().secondaryCurrentLimit(limit);
        sparkMax.applyConfiguration(sparkMax.getSparkConfiguration());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motor velocity", sparkMax::getVelocity);
    }
}
