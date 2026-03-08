package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Kicker extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "transport";

    public static final double SPEED  = -1;

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
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motor velocity",sparkMax::getVelocity);
    }
}
