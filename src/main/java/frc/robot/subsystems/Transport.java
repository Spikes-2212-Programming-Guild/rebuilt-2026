package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Transport extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "transport";

    public static final double SPEED = -1.0;

    private static final double CURRENT_LIMIT_AMP = 40.0;

    private final SparkWrapper sparkMax;

    private static Transport instance;

    public static Transport getInstance() {
        if (instance == null) {
            instance = new Transport(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.TRANSPORT_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private Transport(String namespaceName, SparkWrapper sparkMax) {
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
