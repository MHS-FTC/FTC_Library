package org.firstinspires.ftc.teamcode.FTC_Library.Examples.Autonomous.Modules;

import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Module;

import java.util.Random;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Simple drive for time module
 */

public class RandomMultiPossibility extends Module {
    @Override
    public void start() {

    }

    @Override
    public boolean tick() {
        return true;
    }

    @Override
    public int stop() {
        //picks one of two next steps
        return new Random().nextInt(1);
    }
}
