package frc.robot.commands.difficultCommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Hood;

public class ResetHoodPose extends MoveGenericSubsystem {

    private static final double HOOD_RESET_SPEED = -1.0;

    public ResetHoodPose(Hood hood) {
        super(hood, HOOD_RESET_SPEED);
    }
}
