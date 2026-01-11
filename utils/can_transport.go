package utils

import (
	"context"
	"fmt"
	"net"

	"go.einride.tech/can"
	"go.einride.tech/can/pkg/socketcan"
)

type CANWriter interface {
	WriteFrame(ctx context.Context, frame can.Frame) error
	Close() error
}

type SocketCANWriter struct {
	conn net.Conn
	tx   *socketcan.Transmitter
}

func NewSocketCANWriter(ctx context.Context, iface string) (*SocketCANWriter, error) {
	// net = "can", address = "vcan0" etc. :contentReference[oaicite:3]{index=3}
	conn, err := socketcan.DialContext(ctx, "can", iface)
	if err != nil {
		return nil, fmt.Errorf("socketcan dial: %w", err)
	}
	return &SocketCANWriter{
		conn: conn,
		tx:   socketcan.NewTransmitter(conn),
	}, nil
}

func (w *SocketCANWriter) WriteFrame(ctx context.Context, frame can.Frame) error {
	// tx.TransmitFrame(ctx, frame) is the documented send pattern. :contentReference[oaicite:4]{index=4}
	return w.tx.TransmitFrame(ctx, frame)
}

func (w *SocketCANWriter) Close() error {
	if w.conn != nil {
		return w.conn.Close()
	}
	return nil
}
