package utils

import (
	"encoding/csv"
	"errors"
	"fmt"
	"io"
	"os"
	"sort"
	"strconv"
	"strings"
)

func LoadCANMap(csvPath string) (*CANMap, error) {
	f, err := os.Open(csvPath)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	r := csv.NewReader(f)
	r.TrimLeadingSpace = true

	header, err := r.Read()
	if err != nil {
		return nil, err
	}

	idx := make(map[string]int, len(header))
	for i, h := range header {
		idx[strings.TrimSpace(h)] = i
	}

	req := []string{
		"direction", "priority", "pgn", "sa", "da", "frame_name", "cycle_ms", "dlc",
		"signal_name", "start_bit", "bit_length", "endianness",
		"signed", "factor", "offset", "min", "max", "default", "unit", "comment",
	}
	for _, k := range req {
		if _, ok := idx[k]; !ok {
			return nil, fmt.Errorf("can_map.csv missing required column: %q", k)
		}
	}

	m := &CANMap{
		ByID:   map[uint32]*FrameDef{},
		ByName: map[string]*FrameDef{},
	}

	for {
		rec, err := r.Read()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			return nil, err
		}

		priority, err := parseHexOrDecUint8(rec[idx["priority"]])
		if err != nil {
			return nil, fmt.Errorf("invalid priority %q: %w", rec[idx["priority"]], err)
		}
		pgn, err := parseHexOrDecUint32(rec[idx["pgn"]])
		if err != nil {
			return nil, fmt.Errorf("invalid pgn %q: %w", rec[idx["pgn"]], err)
		}
		sa, err := parseHexOrDecUint8(rec[idx["sa"]])
		if err != nil {
			return nil, fmt.Errorf("invalid sa %q: %w", rec[idx["sa"]], err)
		}
		da, err := parseHexOrDecUint8(rec[idx["da"]])
		if err != nil {
			return nil, fmt.Errorf("invalid da %q: %w", rec[idx["da"]], err)
		}
		frameID := j1939CanID(priority, pgn, sa, da)

		frameName := strings.TrimSpace(rec[idx["frame_name"]])
		direction := strings.TrimSpace(rec[idx["direction"]])

		cycleMS := mustInt(rec[idx["cycle_ms"]])
		dlc := mustInt(rec[idx["dlc"]])

		sig := SignalDef{
			Name:       strings.TrimSpace(rec[idx["signal_name"]]),
			StartBit:   mustInt(rec[idx["start_bit"]]),
			BitLength:  mustInt(rec[idx["bit_length"]]),
			Endianness: strings.TrimSpace(rec[idx["endianness"]]),
			Signed:     mustBool(rec[idx["signed"]]),
			Factor:     mustFloat(rec[idx["factor"]]),
			Offset:     mustFloat(rec[idx["offset"]]),
			Min:        mustFloat(rec[idx["min"]]),
			Max:        mustFloat(rec[idx["max"]]),
			Default:    mustFloat(rec[idx["default"]]),
			Unit:       strings.TrimSpace(rec[idx["unit"]]),
			Comment:    strings.TrimSpace(rec[idx["comment"]]),
		}

		if sig.Endianness != "" && sig.Endianness != "little" {
			return nil, fmt.Errorf("frame %s signal %s: unsupported endianness %q (only little supported)",
				frameName, sig.Name, sig.Endianness)
		}
		if sig.BitLength <= 0 || sig.BitLength > 64 {
			return nil, fmt.Errorf("frame %s signal %s: invalid bit_length %d", frameName, sig.Name, sig.BitLength)
		}
		if dlc <= 0 || dlc > 8 {
			return nil, fmt.Errorf("frame %s (0x%X): invalid dlc %d", frameName, frameID, dlc)
		}

		fd, ok := m.ByID[frameID]
		if !ok {
			fd = &FrameDef{
				ID:         frameID,
				IsExtended: true,
				Priority:   priority,
				PGN:        pgn,
				SA:         sa,
				DA:         da,
				Name:       frameName,
				DLC:        dlc,
				Direction:  direction,
				CycleMS:    cycleMS,
				Signals:    []SignalDef{},
			}
			m.ByID[frameID] = fd
			m.ByName[frameName] = fd
		}

		if fd.DLC != dlc {
			return nil, fmt.Errorf("frame %s (0x%X) has inconsistent DLC (%d vs %d)", frameName, frameID, fd.DLC, dlc)
		}

		fd.Signals = append(fd.Signals, sig)
	}

	for _, fd := range m.ByID {
		sort.Slice(fd.Signals, func(i, j int) bool { return fd.Signals[i].StartBit < fd.Signals[j].StartBit })
	}

	return m, nil
}

func (m *CANMap) FrameByName(name string) (*FrameDef, error) {
	fd, ok := m.ByName[name]
	if !ok {
		return nil, fmt.Errorf("unknown frame %q (available: %v)", name, m.FrameNames())
	}
	return fd, nil
}

func (m *CANMap) FrameByID(id uint32) (*FrameDef, error) {
	fd, ok := m.ByID[id]
	if !ok {
		return nil, fmt.Errorf("unknown frame id 0x%X", id)
	}
	return fd, nil
}

func parseHexOrDecUint32(s string) (uint32, error) {
	ss := strings.TrimSpace(s)
	base := 10
	if strings.HasPrefix(ss, "0x") || strings.HasPrefix(ss, "0X") {
		base = 16
		ss = ss[2:]
	}
	u, err := strconv.ParseUint(ss, base, 32)
	if err != nil {
		return 0, err
	}
	return uint32(u), nil
}

func parseHexOrDecUint8(s string) (uint8, error) {
	ss := strings.TrimSpace(s)
	base := 10
	if strings.HasPrefix(ss, "0x") || strings.HasPrefix(ss, "0X") {
		base = 16
		ss = ss[2:]
	}
	u, err := strconv.ParseUint(ss, base, 8)
	if err != nil {
		return 0, err
	}
	return uint8(u), nil
}

// j1939CanID computes the 29-bit J1939 CAN ID from its components.
// For PDU1 messages (PF < 0xF0): the destination address (da) occupies
// the PS field. For PDU2 messages (PF >= 0xF0): PS comes from the low
// byte of the PGN (group extension).
func j1939CanID(priority uint8, pgn uint32, sa, da uint8) uint32 {
	pf := uint8((pgn >> 8) & 0xFF)
	var ps uint8
	if pf < 0xF0 { // PDU1: peer-to-peer, PS = destination address
		ps = da
	} else { // PDU2: broadcast, PS = group extension from PGN
		ps = uint8(pgn & 0xFF)
	}
	return (uint32(priority) << 26) | (uint32(pf) << 16) | (uint32(ps) << 8) | uint32(sa)
}

func mustInt(s string) int {
	v, _ := strconv.Atoi(strings.TrimSpace(s))
	return v
}

func mustFloat(s string) float64 {
	v, _ := strconv.ParseFloat(strings.TrimSpace(s), 64)
	return v
}

func mustBool(s string) bool {
	ss := strings.TrimSpace(strings.ToLower(s))
	return ss == "true" || ss == "1" || ss == "yes"
}
