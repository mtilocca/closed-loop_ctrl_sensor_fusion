package utils

import "sort"

type SignalDef struct {
	Name       string
	StartBit   int
	BitLength  int
	Signed     bool
	Factor     float64
	Offset     float64
	Min        float64
	Max        float64
	Default    float64
	Unit       string
	Comment    string
	Endianness string // only "little" supported
}

type FrameDef struct {
	ID         uint32 // computed 29-bit J1939 CAN ID
	IsExtended bool   // true for J1939 (29-bit) frames
	Priority   uint8
	PGN        uint32
	SA         uint8 // source address
	DA         uint8 // destination address (PDU1) or 0xFF (PDU2)
	Name       string
	DLC        int
	Direction  string
	CycleMS    int
	Signals    []SignalDef
}

type CANMap struct {
	ByID   map[uint32]*FrameDef
	ByName map[string]*FrameDef
}

func (m *CANMap) FrameNames() []string {
	out := make([]string, 0, len(m.ByName))
	for k := range m.ByName {
		out = append(out, k)
	}
	sort.Strings(out)
	return out
}
