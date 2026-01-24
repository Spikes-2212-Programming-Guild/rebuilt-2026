package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Transport extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "transport";


    private final SparkWrapper Motor;
    private static Transport instance;

    public static Transport getInstance() {
        if (instance == null) {
            instance = new Transport(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.TRANSPORT_NEO_ID, SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private Transport(String namespaceName, SparkWrapper motor) {
        super(namespaceName, motor);
        this.Motor = motor;
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber(" velocity ", Motor::getVelocity);
    }
}
