package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import static frc.robot.RobotMap.CAN.SPINNING_MAGAZINE_SPARK_MAX_ID;

public class SpinningMagazine extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "spinning magazine";

    private final SparkWrapper motor;

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(SPINNING_MAGAZINE_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private SpinningMagazine(String namespaceName, SparkWrapper motor){
        super(namespaceName, motor);
        this.motor = motor;
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("motor speed",
                motor::getVelocity);
    }
}
