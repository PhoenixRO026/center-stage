package com.phoenix_ro026.phoenixlib.old_units.java

import com.phoenix_ro026.phoenixlib.old_units.degSec
import com.phoenix_ro026.phoenixlib.old_units.radPerSec
import com.phoenix_ro026.phoenixlib.old_units.rpm

fun rpm(x: Double) = x.rpm.toUnit(radPerSec)
fun radSec(x: Double) = x
fun degSec(x: Double) = x.degSec.toUnit(radPerSec)