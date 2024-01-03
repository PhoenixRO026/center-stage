package com.phoenix_ro026.phoenixlib.units.java

import com.phoenix_ro026.phoenixlib.units.cm
import com.phoenix_ro026.phoenixlib.units.meters
import com.phoenix_ro026.phoenixlib.units.mm

fun cm(x: Double) = x.cm.toInches()
fun mm(x: Double) = x.mm.toInches()
fun inch(x: Double) = x
fun meters(x: Double) = x.meters.toInches()