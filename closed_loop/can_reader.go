//go:build linux || darwin
// +build linux darwin

package main

import (
	"context"
	"fmt"
	"net"

	"go.einride.tech/can"
	"go.einride.tech/can/pkg/socketcan"
)

// CANReader defines the interface for reading CAN frames
type CANReader interface {
	ReadFrame(ctx context.Context) (can.Frame, error)
	Close() error
}

// SocketCANReader implements CANReader using Einride's socketcan
type SocketCANReader struct {
	conn net.Conn
	recv *socketcan.Receiver
}

// NewSocketCANReader creates a new SocketCAN reader
func NewSocketCANReader(ctx context.Context, ifname string) (*SocketCANReader, error) {
	// Use Einride's DialContext (same as can_transport.go)
	conn, err := socketcan.DialContext(ctx, "can", ifname)
	if err != nil {
		return nil, fmt.Errorf("socketcan dial: %w", err)
	}

	recv := socketcan.NewReceiver(conn)

	return &SocketCANReader{
		conn: conn,
		recv: recv,
	}, nil
}

// ReadFrame reads a single CAN frame (blocking)
func (r *SocketCANReader) ReadFrame(ctx context.Context) (can.Frame, error) {
	// Channel to receive frame
	frameChan := make(chan can.Frame, 1)
	errChan := make(chan error, 1)

	// Read in goroutine to support context cancellation
	go func() {
		if r.recv.Receive() {
			frameChan <- r.recv.Frame()
		} else {
			errChan <- fmt.Errorf("receive failed")
		}
	}()

	// Wait for frame or context cancellation
	select {
	case <-ctx.Done():
		return can.Frame{}, ctx.Err()
	case frame := <-frameChan:
		return frame, nil
	case err := <-errChan:
		return can.Frame{}, err
	}
}

// Close closes the CAN socket
func (r *SocketCANReader) Close() error {
	if r.conn != nil {
		return r.conn.Close()
	}
	return nil
}
