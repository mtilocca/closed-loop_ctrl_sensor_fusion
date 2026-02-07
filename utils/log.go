package utils

import (
	"fmt"
	"os"
	"sync"
	"time"
)

type LogLevel int

const (
	TRACE LogLevel = iota
	DEBUG
	INFO
	WARN
	ERROR
	CRITICAL
)

func (l LogLevel) String() string {
	switch l {
	case TRACE:
		return "TRACE"
	case DEBUG:
		return "DEBUG"
	case INFO:
		return "INFO"
	case WARN:
		return "WARN"
	case ERROR:
		return "ERROR"
	case CRITICAL:
		return "CRITICAL"
	default:
		return "UNKNOWN"
	}
}

/*
ANSI color codes (stdout only)
*/
const (
	colorReset   = "\033[0m"
	colorGray    = "\033[90m"
	colorBlue    = "\033[34m"
	colorGreen   = "\033[32m"
	colorYellow  = "\033[33m"
	colorRed     = "\033[31m"
	colorMagenta = "\033[35m"
)

func levelColor(level LogLevel) string {
	switch level {
	case TRACE:
		return colorGray
	case DEBUG:
		return colorBlue
	case INFO:
		return colorGreen
	case WARN:
		return colorYellow
	case ERROR:
		return colorRed
	case CRITICAL:
		return colorMagenta
	default:
		return colorReset
	}
}

type Logger struct {
	mu         sync.Mutex
	minLevel   LogLevel
	filePath   string
	file       *os.File
	alsoStdout bool
}

func NewFileLogger(filePath string, minLevel LogLevel, alsoStdout bool) (*Logger, error) {
	f, err := os.OpenFile(filePath, os.O_CREATE|os.O_APPEND|os.O_WRONLY, 0644)
	if err != nil {
		return nil, err
	}
	return &Logger{
		minLevel:   minLevel,
		filePath:   filePath,
		file:       f,
		alsoStdout: alsoStdout,
	}, nil
}

func (l *Logger) Close() error {
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.file != nil {
		return l.file.Close()
	}
	return nil
}

func (l *Logger) SetMinLevel(level LogLevel) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.minLevel = level
}

func (l *Logger) log(level LogLevel, msg string, args ...any) {
	l.mu.Lock()
	defer l.mu.Unlock()

	if level < l.minLevel {
		return
	}

	ts := time.Now().Format(time.RFC3339Nano)
	rawLine := fmt.Sprintf(
		"%s [%s] %s\n",
		ts,
		level.String(),
		fmt.Sprintf(msg, args...),
	)

	// File logging (no colors)
	if l.file != nil {
		_, _ = l.file.WriteString(rawLine)
		_ = l.file.Sync()
	}

	// Stdout logging (with colors)
	if l.alsoStdout {
		color := levelColor(level)
		coloredLine := fmt.Sprintf("%s%s%s", color, rawLine, colorReset)
		_, _ = os.Stdout.WriteString(coloredLine)
	}
}

func (l *Logger) Trace(msg string, args ...any)    { l.log(TRACE, msg, args...) }
func (l *Logger) Debug(msg string, args ...any)    { l.log(DEBUG, msg, args...) }
func (l *Logger) Info(msg string, args ...any)     { l.log(INFO, msg, args...) }
func (l *Logger) Warn(msg string, args ...any)     { l.log(WARN, msg, args...) }
func (l *Logger) Error(msg string, args ...any)    { l.log(ERROR, msg, args...) }
func (l *Logger) Critical(msg string, args ...any) { l.log(CRITICAL, msg, args...) }
