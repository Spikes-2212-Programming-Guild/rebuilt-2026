package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Collection extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "collection";

    private final SparkWrapper spark;

    public static final double SPEED = -1.0;

    private final double CURRENT_LIMIT_AMP = 40;

    private static Collection instance;

    public static Collection getInstance() {
        if (instance == null) {
            instance = new Collection(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.COLLECTION_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private Collection(String namespaceName, SparkWrapper spark) {
        super(namespaceName, spark);
        this.spark = spark;
        configureDashboard();
        setCurrentLimit(CURRENT_LIMIT_AMP);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motor velocity", spark::getVelocity);
    }

    public void setCurrentLimit(double limit) {
        spark.getSparkConfiguration().secondaryCurrentLimit(limit);
        spark.applyConfiguration(spark.getSparkConfiguration());
    }
}
