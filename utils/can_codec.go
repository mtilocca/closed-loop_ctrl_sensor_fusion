package utils

import (
	"fmt"
	"math"

	"go.einride.tech/can"
)

func (m *CANMap) EncodeFrame(frameName string, values map[string]float64) ([]byte, uint32, error) {
	fd, err := m.FrameByName(frameName)
	if err != nil {
		return nil, 0, err
	}
	if fd.DLC <= 0 || fd.DLC > 8 {
		return nil, 0, fmt.Errorf("frame %s has invalid DLC %d", fd.Name, fd.DLC)
	}

	var payload uint64 = 0

	for _, s := range fd.Signals {
		v, ok := values[s.Name]
		if !ok {
			v = s.Default
		}

		v = clamp(v, s.Min, s.Max)

		rawFloat := (v - s.Offset) / s.Factor
		raw := int64(math.Round(rawFloat))
		raw = clampRaw(raw, s.BitLength, s.Signed)

		u := uint64(rawToUnsigned(raw, s.BitLength))
		payload = setBits(payload, s.StartBit, s.BitLength, u)
	}

	out := make([]byte, fd.DLC)
	for i := 0; i < fd.DLC; i++ {
		out[i] = byte((payload >> (8 * i)) & 0xFF)
	}
	return out, fd.ID, nil
}

// Helper: produce einride can.Frame ready to transmit.
func (m *CANMap) EncodeEinrideFrame(frameName string, values map[string]float64) (can.Frame, error) {
	payload, id, err := m.EncodeFrame(frameName, values)
	if err != nil {
		return can.Frame{}, err
	}

	var f can.Frame
	f.ID = id              // <-- FIX: assign uint32 directly
	f.Length = uint8(len(payload))
	copy(f.Data[:], payload)

	return f, nil
}


func (m *CANMap) DecodeFrame(frameID uint32, data []byte) (map[string]float64, error) {
	fd, err := m.FrameByID(frameID)
	if err != nil {
		return nil, err
	}
	if len(data) < fd.DLC {
		return nil, fmt.Errorf("frame 0x%X expects DLC %d, got %d", frameID, fd.DLC, len(data))
	}

	var payload uint64 = 0
	for i := 0; i < fd.DLC && i < 8; i++ {
		payload |= uint64(data[i]) << (8 * i)
	}

	out := make(map[string]float64, len(fd.Signals))
	for _, s := range fd.Signals {
		u := getBits(payload, s.StartBit, s.BitLength)
		raw := unsignedToRawInt64(u, s.BitLength, s.Signed)
		phys := float64(raw)*s.Factor + s.Offset
		out[s.Name] = phys
	}
	return out, nil
}
