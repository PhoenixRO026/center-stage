package com.phoenix_ro026.phoenixlib.old_units.java

import com.phoenix_ro026.phoenixlib.old_units.cm
import com.phoenix_ro026.phoenixlib.old_units.meters
import com.phoenix_ro026.phoenixlib.old_units.mm

fun cm(x: Double) = x.cm.toInches()
fun mm(x: Double) = x.mm.toInches()
fun inch(x: Double) = x
fun meters(x: Double) = x.meters.toInches()