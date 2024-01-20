package com.phoenix_ro026.phoenixlib;

import static com.phoenix_ro026.phoenixlib.units.AngleKt.deg;
import static com.phoenix_ro026.phoenixlib.units.DistanceKt.cm;

import com.phoenix_ro026.phoenixlib.units.Angle;
import com.phoenix_ro026.phoenixlib.units.Centimeters;
import com.phoenix_ro026.phoenixlib.units.Degrees;
import com.phoenix_ro026.phoenixlib.units.Distance;
import com.phoenix_ro026.phoenixlib.units.Pose;

public class ApiTest {

    public void runOpMode() throws InterruptedException {
        Pose<Distance, Distance, Angle> test = new Pose<>(cm(2), cm(2), deg(2));
    }
}
