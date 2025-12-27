package utils

func getBits(payload uint64, startBit, bitLen int) uint64 {
	if bitLen <= 0 || bitLen > 64 {
		return 0
	}
	mask := uint64((1 << bitLen) - 1)
	return (payload >> startBit) & mask
}

func setBits(payload uint64, startBit, bitLen int, value uint64) uint64 {
	if bitLen <= 0 || bitLen > 64 {
		return payload
	}
	mask := uint64((1 << bitLen) - 1)
	payload &^= (mask << startBit)
	payload |= (value & mask) << startBit
	return payload
}

func unsignedToRawInt64(u uint64, bitLen int, signed bool) int64 {
	if !signed {
		return int64(u)
	}
	signBit := uint64(1) << (bitLen - 1)
	if (u & signBit) == 0 {
		return int64(u)
	}
	fullMask := uint64((1 << bitLen) - 1)
	twos := (^u + 1) & fullMask
	return -int64(twos)
}

func rawToUnsigned(raw int64, bitLen int) uint64 {
	if raw >= 0 {
		return uint64(raw)
	}
	fullMask := uint64((1 << bitLen) - 1)
	u := uint64(-raw)
	twos := (^u + 1) & fullMask
	return twos
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func clampRaw(raw int64, bitLen int, signed bool) int64 {
	if bitLen <= 0 || bitLen > 63 {
		return raw
	}
	if !signed {
		max := int64((1 << bitLen) - 1)
		if raw < 0 {
			return 0
		}
		if raw > max {
			return max
		}
		return raw
	}
	min := -int64(1 << (bitLen - 1))
	max := int64((1 << (bitLen - 1)) - 1)
	if raw < min {
		return min
	}
	if raw > max {
		return max
	}
	return raw
}
