# Rerun Bridge - Real-Time Robot Visualization

Welcome! This directory contains the **Rerun Bridge** — a real-time 3D visualization system for the DaVinciBot Eurobot 2026 robot.

**What is Rerun?** It's an open-source visualization framework that displays robot state (position, LiDAR, path, etc.) in a web browser in real-time.

---

## 📖 Documentation

Start with **one of these** based on your needs:

### 🚀 **New to Rerun?** Start here:
→ **[RERUN_QUICK_START.md](RERUN_QUICK_START.md)** (5 minutes)
- Launch Rerun bridge
- View robot in browser  
- Verify it's working
- Add to your code

### 🔧 **Integrating into robot code?**
→ **[RERUN_INTEGRATION_EXAMPLES.md](RERUN_INTEGRATION_EXAMPLES.md)** (Copy-paste examples)
- Minimal integration (5 lines)
- Full integration (pathfinding, LiDAR)
- Testing without hardware
- Error handling patterns

### 🏗️ **Need detailed explanation?**
→ **[RERUN_DATA_FLOW_INTEGRATION.md](RERUN_DATA_FLOW_INTEGRATION.md)** (Complete guide)
- Architecture overview
- How data flows through system
- Thread-safe state management
- Performance tuning
- Auto-launch on boot

### 🧠 **Want technical deep-dive?**
→ **[RERUN_ARCHITECTURE_TECHNICAL.md](RERUN_ARCHITECTURE_TECHNICAL.md)** (Advanced)
- System architecture diagrams
- Complete data flow paths
- Thread model explanation
- API reference
- Protocol specifications
- Extensibility guide

---

## 🎯 Quick Links

| Task | Link |
|------|------|
| **Start bridge + open viewer** | [Quick Start §1-3](RERUN_QUICK_START.md#1-prerequisites) |
| **Verify robot position updates** | [Quick Start §4](RERUN_QUICK_START.md#4-verify-robot-position) |
| **Add to robot.py** | [Quick Start §5](RERUN_QUICK_START.md#5-add-to-robotpy-5-minute-integration) |
| **Full integration example** | [Examples §2](RERUN_INTEGRATION_EXAMPLES.md#example-2-full-integration-with-lidar--pathfinding) |
| **Understand data flow** | [Architecture §2](RERUN_DATA_FLOW_INTEGRATION.md#how-data-flows-real-example) |
| **Setup auto-launch** | [Data Flow §9](RERUN_DATA_FLOW_INTEGRATION.md#auto-launch-on-raspberry-pi-boot) |
| **Troubleshoot issues** | [Quick Start §7](RERUN_QUICK_START.md#7-troubleshooting) |
| **Performance tuning** | [Data Flow §7](RERUN_DATA_FLOW_INTEGRATION.md#performance-notes) |

---

## 📊 What Gets Visualized

### 3D Terrain View
- 🔵 **Blue cylinder**: Teensy odometry (primary)
- 🔴 **Red cylinder**: LiDAR localization (when confident)
- 🟢 **Green cylinder**: Fused best estimate (recommended)
- 🟡 **Yellow point + arrow**: Navigation target (with direction)
- 🔶 **Orange diamonds**: Detected beacon markers
- 🔴 **Red spheres**: Obstacles & adversary robots
- ⚪ **Gray cylinders**: Fixed field beacons
- 📦 **Boxes/meshes**: Walls, field layout
- 🪶 **Dotted line**: Planned trajectory

### 2D Polar View (LiDAR Radar)
- 🟢 **Green rays**: LiDAR distance measurements
- 🔵 **Blue circles**: Reference grid (500mm rings)

### Time Series Plots
- **Teensy**: Position (X, Y, θ) over time
- **LiDAR**: Confidence, point count, beacon count
- **Fused**: Best position estimate (X, Y)
- **Fusion error**: Distance between odometry & LiDAR

---

## 🏃 Quick Start (60 seconds)

### Terminal 1: Start Bridge (on Raspberry Pi)
```bash
cd /robot1/rasp/rerun
python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar
```

### Terminal 2: Open Viewer (on PC)
```
Open browser: http://192.168.1.50:9876
(Replace IP with actual Rasp IP)
```

### Expected Result
- ✅ 3D terrain loads
- ✅ Blue cylinder visible (robot)
- ✅ Blue cylinder moves as robot moves
- ✅ LiDAR polar view updates
- ✅ Time series plots show smooth curves

👉 **Next:** Add to your robot code — [See Examples](RERUN_INTEGRATION_EXAMPLES.md#example-1-minimal-integration-in-robotpy)

---

## 📁 File Structure

```
robot1/rasp/rerun/
│
├── rerun_bridge.py                           ← Main bridge (DO NOT EDIT)
│   • State management (_State class)
│   • Public API (update_* functions)
│   • Rendering loop (publish_loop)
│   • Network server (serve/connect/local modes)
│
├── RERUN_QUICK_START.md                      ← Start here (5 min)
├── RERUN_INTEGRATION_EXAMPLES.md             ← Code examples (10 min)
├── RERUN_DATA_FLOW_INTEGRATION.md            ← Full guide (30 min)
├── RERUN_ARCHITECTURE_TECHNICAL.md           ← Deep dive (45 min)
├── README.md                                 ← This file
│
├── map_assets/
│   └── eurobot2026/
│       ├── meshes/                           ← 3D models (.ply)
│       ├── textures/                         ← Textures (.jpg/.png)
│       └── ...
│
└── test/
    └── test_rerun_integration.py             ← Simulation test
```

---

## 🔌 Integration Points

### Data Sources
| Source | Update Function | Frequency | Example |
|--------|-----------------|-----------|---------|
| **Teensy (USB)** | `update_odom(x,y,θ)` | 10 Hz | Position from encoders + IMU |
| **LiDAR** | `update_lidar_cloud(pts)` | 20 Hz | Raw distance measurements |
| **LiDAR** | `update_lidar_beacons(cands)` | 20 Hz | Detected beacon positions |
| **LiDAR** | `update_lidar_pose(x,y,θ,conf,ok)` | 20 Hz | Localization estimate |
| **PathFinding** | `update_target(x,y)` | Variable | Current goal |
| **PathFinding** | `update_trajectory(pts)` | Variable | Planned path |
| **Fusion** | `update_fused(x,y,θ)` | 20 Hz | Best position estimate |

### Where to Call
- **`update_odom()`** ← USB callback (automatic)
- **`update_lidar_*()`** ← Polling loop (automatic)
- **`update_target()`** ← Your strategy code when goal changes
- **`update_trajectory()`** ← Your pathfinder code after replanning
- **`update_fused()`** ← Your robot.py after fusion computation

---

## 💻 Code Example (3 lines to add)

**Before:**
```python
# robot.py
def update(self):
    x, y, theta = self.x, self.y, self.theta
    # ... process ...
```

**After:**
```python
# robot.py (add 2 lines at top of file)
from rerun_bridge import update_fused

def update(self):
    x, y, theta = self.x, self.y, self.theta
    update_fused(x, y, theta)  # ← Add this
    # ... process ...
```

**That's it!** Now open http://RaspIP:9876 in browser and watch your robot move.

For more examples → [See Examples](RERUN_INTEGRATION_EXAMPLES.md)

---

## 🎮 Modes

### Local Mode (Development on Rasp with display)
```bash
python rerun_bridge.py --mode local --sim
# → Viewer opens directly on Rasp
```

### Serve Mode (WiFi access from PC) ⭐ **RECOMMENDED**
```bash
python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar
# → Open: http://RaspIP:9876 in PC browser
```

### Connect Mode (Remote Rerun Viewer)
```bash
# On external PC:
rerun --serve --port 8812

# On Rasp:
python rerun_bridge.py --mode connect --host PC_IP --port 8812
```

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| "Connection refused" | Check Rasp IP: `hostname -I` on Rasp |
| "Position not updating" | Verify Teensy connected: `lsusb` on Rasp |
| "LiDAR not showing" | Launch with `--with-lidar` flag |
| "High CPU usage" | Use `--mode local` or reduce `hz=10` |
| "Viewer shows stale data" | Restart bridge: `pkill -f rerun_bridge` |

→ **More help:** [Troubleshooting guide](RERUN_QUICK_START.md#7-troubleshooting)

---

## 📈 Performance

| Metric | Value | Notes |
|--------|-------|-------|
| Publishing rate | 20 Hz | 50 ms per frame |
| Network bandwidth | ~500 KB/s | At 20 Hz on LAN |
| CPU overhead (Rasp Pi 4) | ~15% | When all systems active |
| Memory usage | ~50-60 MB | Stable, no leaks |
| Latency (USB to screen) | ~80-100 ms | LAN, modern browser |

→ **Optimization tips:** [Performance notes](RERUN_DATA_FLOW_INTEGRATION.md#performance-notes)

---

## 🚀 Getting Started Checklist

- [ ] Launch bridge: `python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876`
- [ ] Open browser: `http://RaspIP:9876`
- [ ] See robot (blue cylinder) move as you drive
- [ ] Read: [Quick Start](RERUN_QUICK_START.md)
- [ ] Integrate into code: [Examples](RERUN_INTEGRATION_EXAMPLES.md)
- [ ] Understand architecture: [Data Flow](RERUN_DATA_FLOW_INTEGRATION.md)
- [ ] Deploy to production: [Auto-launch](RERUN_DATA_FLOW_INTEGRATION.md#auto-launch-on-raspberry-pi-boot)

---

## 📚 Documentation Map

```
README.md (you are here)
│
├─→ QUICK_START.md ........... 5 min, hands-on
│   │
│   ├─→ INTEGRATION_EXAMPLES.md  10 min, code samples
│   │
│   ├─→ DATA_FLOW.md .......... 30 min, complete guide
│   │   │
│   │   └─→ ARCHITECTURE_TECHNICAL.md 45 min, deep dive
│   │
│   └─→ §14 Help & Support
│
└─→ Directory structure
    ├─ rerun_bridge.py
    ├─ map_assets/
    └─ test/
```

---

## 🔗 Important Links

- 📖 **Rerun Docs:** https://rerun.io/docs
- 🐛 **Report Issues:** GitHub issues (CDR-Nantes repo)
- 💬 **Team Chat:** Slack #eurobot-visualization
- 📋 **Main README:** [../README.md](../README.md)
- 📊 **Architecture Docs:** [../ODOMETRY_CORRECTION_IMPLEMENTATION.md](../ODOMETRY_CORRECTION_IMPLEMENTATION.md)

---

## 👥 Contributing

**Want to extend Rerun visualization?**

1. Add new field to `_State` class
2. Add public `update_*()` function
3. Add rendering in `_publish()`
4. Add to documentation

→ See [Extensibility guide](RERUN_ARCHITECTURE_TECHNICAL.md#extensibility)

---

## 📝 License

Same as main CDR-Nantes repo (see LICENSE file)

---

## ❓ FAQ

**Q: Does Rerun slow down the robot?**
A: No, it runs in separate thread. ~15% CPU overhead on Rasp Pi 4. Can be disabled if needed.

**Q: Can I view from PC without being on same network?**
A: Yes, use SSH port forwarding: `ssh -L 9876:localhost:9876 pi@rasps_ip`

**Q: What if Teensy USB is not connected?**
A: Bridge falls back to LiDAR-only mode or simulation (with `--sim`)

**Q: Can I customize the visualization?**
A: Yes! See [Extensibility section](RERUN_ARCHITECTURE_TECHNICAL.md#extensibility)

**Q: How do I record sessions?**
A: Rerun viewer auto-records timeline. Use "Save blueprint" in UI to export.

---

**Ready to get started?** → [RERUN_QUICK_START.md](RERUN_QUICK_START.md)

