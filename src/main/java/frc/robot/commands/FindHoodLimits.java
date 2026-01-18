package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Hood;

public class FindHoodLimits extends MoveGenericSubsystem {

    private static final double HOOD_RESET0_SPEED = -1.0;

    private final Hood hood;

    public FindHoodLimits(Hood hood) {
        super(hood, HOOD_RESET0_SPEED);
        this.hood = hood;
        addRequirements(hood);
    }
}
