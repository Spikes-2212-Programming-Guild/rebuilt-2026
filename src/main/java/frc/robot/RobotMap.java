package frc.robot;

public class RobotMap {

    public interface CAN {

        int COLLECTION_TALON_FX_ID = 14;

        int COLLECTION_MOVEMENT_TALON_FX_ID = 15;

        int SPINNING_MAGAZINE_TALON_FX_ID = 16;

        int TRANSPORT_SPARK_MAX_ID = 17;

        int HOOD_SPARK_MAX = 18;

        int SHOOTER_LOWER_TALON_FX_ID = 19;
        int SHOOTER_MIDDLE_TALON_FX_ID = 20;
        int SHOOTER_UPPER_TALON_FX_ID = 21;
    }

    public interface DIO {

        int HOOD_ABSOLUTE_ENCODER = -1;

    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

    }
}
