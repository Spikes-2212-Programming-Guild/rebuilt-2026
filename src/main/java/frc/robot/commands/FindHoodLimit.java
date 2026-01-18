package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Hood;

public class FindHoodLimit extends MoveGenericSubsystem {

    private static final double HOOD_RESET0_SPEED = -1.0;

    public FindHoodLimit(Hood hood) {
        super(hood, HOOD_RESET0_SPEED);
        addRequirements(hood);
    }
}
