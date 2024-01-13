package com.phoenix_ro026.phoenixlib.old_units.java

import com.phoenix_ro026.phoenixlib.old_units.cmSec
import com.phoenix_ro026.phoenixlib.old_units.inchPerSec
import com.phoenix_ro026.phoenixlib.old_units.meterSec

fun cmSec(x: Double) = x.cmSec.toUnit(inchPerSec)
fun inchSec(x: Double) = x
fun meterSec(x: Double) = x.meterSec.toUnit(inchPerSec)