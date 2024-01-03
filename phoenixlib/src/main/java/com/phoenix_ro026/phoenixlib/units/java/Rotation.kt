package com.phoenix_ro026.phoenixlib.units.java

import com.phoenix_ro026.phoenixlib.units.deg
import com.phoenix_ro026.phoenixlib.units.rev

fun deg(x: Double) = x.deg.toRadians()
fun rad(x: Double) = x
fun rev(x: Double) = x.rev.toRadians()