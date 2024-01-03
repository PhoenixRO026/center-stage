package com.phoenix_ro026.phoenixlib.units.java

import com.phoenix_ro026.phoenixlib.units.degSec
import com.phoenix_ro026.phoenixlib.units.radPerSec
import com.phoenix_ro026.phoenixlib.units.rpm

fun rpm(x: Double) = x.rpm.toUnit(radPerSec)
fun radSec(x: Double) = x
fun degSec(x: Double) = x.degSec.toUnit(radPerSec)