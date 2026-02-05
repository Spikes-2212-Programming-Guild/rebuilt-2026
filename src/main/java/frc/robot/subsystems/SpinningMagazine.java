package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class SpinningMagazine extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "spinning magazine";

    private final SparkWrapper sparkMax;

    public static final double SPEED = -1.0;

    private final double CURRENT_AMP_LIMIT = 40;

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.SPINNING_MAGAZINE_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private SpinningMagazine(String namespaceName, SparkWrapper sparkMax) {
        super(namespaceName, sparkMax);
        this.sparkMax = sparkMax;
        configureDashboard();
        setCurrentLimit(CURRENT_AMP_LIMIT);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("sparkMax speed", sparkMax::getVelocity);
    }

    public void setCurrentLimit(double limit) {
        sparkMax.getSparkConfiguration().secondaryCurrentLimit(limit);
        sparkMax.applyConfiguration(sparkMax.getSparkConfiguration());
    }
}
