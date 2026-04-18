# Teensy Inter-Board Communication Protocol

## Serial1 Communication (teensy_capteur ↔ teensy_moteur)

### Physical Connection
```
teensy_capteur                         teensy_moteur
   TX1 (pin 16) ──────────────────────→ RX1 (pin 17)
   RX1 (pin 17) ←──────────────────────  TX1 (pin 16)
   GND ─────────────────────────────── GND (common)
```

### Baud Rate
- **115200 bps** (standard for Teensy 4.1 Serial1)
- 10 bits per byte (1 start + 8 data + 1 stop)
- ~87 µs per byte

### Packet Format

**Fixed 14-byte packet**:
```
Byte 0     : 0xAA (sync start marker)
Byte 1-4   : dx (float, IEEE 754)
Byte 5-8   : dy (float, IEEE 754)
Byte 9-12  : dtheta (float, IEEE 754)
Byte 13    : 0xBB (sync end marker)
```

**Example (hex)**:
```
AA 00 00 20 42 00 00 10 42 7B 14 06 3E BB
   └─ dx=40.0mm ┘ └─ dy=8.0mm ┘ └─ dtheta=0.134 rad ┘
```

### Data Rates

| Aspect | Value |
|--------|-------|
| Transmission Rate | 100 Hz (every 10ms) |
| Bytes per packet | 14 bytes |
| Data throughput | 140 bytes/sec = 1.12 kbps |
| Link utilization | 1.2% of 115200 bps |
| Time per packet | 0.122 ms at 115200 bps |

## Receiver State Machine (teensy_moteur)

**States**:
1. `RX_SYNC_START`: Waiting for 0xAA marker
2. `RX_DX`: Collecting 4 bytes for dx (float)
3. `RX_DY`: Collecting 4 bytes for dy (float)
4. `RX_DTHETA`: Collecting 4 bytes for dtheta (float)
5. `RX_SYNC_END`: Verifying 0xBB end marker

**Flow**:
```
RX_SYNC_START
     ↓ (byte == 0xAA)
RX_DX (read 4 bytes)
     ↓ (4 bytes collected)
RX_DY (read 4 bytes)
     ↓ (4 bytes collected)
RX_DTHETA (read 4 bytes)
     ↓ (4 bytes collected)
RX_SYNC_END (verify 0xBB)
     ↓ (valid packet)
    [UPDATE POSITION]
     ↓ (invalid or end marker found)
RX_SYNC_START (restart)
```

**Robustness**:
- Non-blocking: continues receiving even if previous byte was invalid
- Auto-recovery: bad sync markers skip to next 0xAA
- No CRC needed for 100 Hz rate (loss tolerance high)

## Odometry Integration (teensy_moteur)

**Every 100 Hz ISR** (`interruption_compute()`):
- Position is read from main loop via non-blocking sensor data
- Delta (dx, dy, dtheta) is accumulated into current position

**Coordinate Transformation**:
```cpp
// Robot frame → World frame
world_dx = dx * cos(THETA) - dy * sin(THETA)
world_dy = dx * sin(THETA) + dy * cos(THETA)

// Update position (meters)
X += world_dx / 1000  // Convert mm to m
Y += world_dy / 1000
THETA += dtheta
```

## Sensor Calibration (teensy_capteur)

### IMU Yaw Offset
- Captured during `setup()` - 500ms window
- Used to zero out initial heading
- Applied to all subsequent quaternion readings

### Optical Sensor Filtering
- First read: ignored (often spurious)
- Magnitude > 15mm: rejected as outlier (physically impossible at 100Hz)
- Magnitude < 2mm: treated as noise, zeroed

## Timing & Synchronization

```
teensy_capteur (100 Hz)          teensy_moteur (100 Hz ISR)
    │                                    │
    ├─ Read PAA5100 (~1ms)               │
    ├─ Read BNO085 (~0.5ms)              │
    ├─ Calculate dx,dy,dθ (~0.5ms)       │
    └─ Send 14 bytes (~0.12ms) ─────────→├─ Receive non-blocking
                                         ├─ Update position
                                         ├─ ISR fires at 100Hz
                                         ├─ Fusion + PID
                                         └─ Motor commands
    
    Total: 10ms cycle          Total: 10ms cycle
```

**Note**: No explicit synchronization needed - both run at 100Hz independently

## Error Handling

### Teensy_capteur
- Optical sensor init timeout: 100ms → continues without optical
- IMU init timeout: 500ms → continues with offset=0
- Serial1 send: fire-and-forget (UDP-like)

### Teensy_moteur
- Serial1 receive: non-blocking, auto-recovery on bad bytes
- Position update: atomic (interrupts disabled)
- Missing packets: use last valid position

## Testing & Debugging

### Monitor teensy_capteur (USB):
```
Serial.print() outputs at 10Hz:
"dx=X.XXX dy=Y.YYY dtheta=Z.ZZZZ | opt_valid=N opt_outlier=M"
```

### Monitor teensy_moteur (USB):
```
Position output at ~5Hz:
"X=XXX.XX Y=YYY.YY THETA=ZZZ.ZZ"
```

### Oscilloscope (TX1 pin analysis):
- Expect 14 bytes every 10ms
- Waveform: standard 8-N-1 UART
- Idle high voltage between packets

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| No data received | Wire not connected | Check TX1/RX1 pins |
| Garbage data | Baud rate mismatch | Verify both 115200 |
| Intermittent drops | Noise on lines | Add 100nF capacitors (GND) |
| Position jumps | Bad sync byte | Check SYNC_START/END values |
| Slow updates | Serial1 buffer full | Increase buffer size |

## Future Enhancements

- [ ] Add CRC8 checksum for reliability
- [ ] Implement adaptive baud rate (e.g., 230400 for faster response)
- [ ] Add quaternion confidence scores from IMU
- [ ] Send IMU raw acceleration/gyro for better fusion
- [ ] Implement timestamp in packets for latency measurement
