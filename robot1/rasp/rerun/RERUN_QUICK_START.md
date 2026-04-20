# Rerun Quick Start Guide

**TL;DR:** 2-minute setup to visualize robot in Rerun.

---

## 1. Prerequisites

✅ **Check these are installed:**
```bash
pip list | grep -i rerun
# Should show: rerun-sdk (version ≥ 0.15.0)
```

If not installed:
```bash
pip install rerun-sdk
```

---

## 2. Start the Rerun Bridge (on Raspberry Pi)

```bash
cd /robot1/rasp/rerun

# For development/testing (on Rasp with X11):
python rerun_bridge.py --mode local --sim

# For WiFi access from PC (RECOMMENDED):
python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar
```

**Output should look like:**
```
✓ Lidar polling activé
🌐 Mode SERVE activé
   Serveur Rerun WebSocket sur 0.0.0.0:9876
   Accédez depuis un navigateur : http://[RaspIP]:9876
▶ Publication Rerun à 20 Hz
```

---

## 3. Open Viewer (on PC Laptop)

**In browser on your PC:**
```
http://192.168.1.50:9876  # Replace with actual Rasp IP
```

**You should see:**
- 3D terrain with robot (blue cylinder)
- LiDAR polar view (radar)
- Time series plots
- Updates in real-time

---

## 4. Verify Robot Position

**Physical Test:**
1. Run the robot in a path on the table
2. Watch the **blue cylinder** move in the Rerun viewer
3. Position should **match robot's physical location**

**If not working:**
- [ ] Check Rasp IP is correct: `hostname -I` on Rasp
- [ ] Check port 9876 is open: `curl http://RaspIP:9876/health`
- [ ] Check USB Teensy connected: `lsusb` on Rasp
- [ ] Check LiDAR connected: `ls /dev/ttyUSB*` on Rasp

---

## 5. Add to robot.py (5 minute integration)

### Step 1: Import at top of file
```python
# robot.py (add at top)
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "rerun"))
try:
    from rerun_bridge import update_fused, update_target, update_trajectory
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False
```

### Step 2: Call in update() loop
```python
def update(self):
    """Main control loop (called 20 Hz)."""
    
    # Existing code...
    with self.lock:
        x, y, theta = self.x, self.y, self.theta
    
    # ... compute positions ...
    
    # Add these 2 lines:
    if RERUN_AVAILABLE:
        update_fused(rx, ry, rtheta)  # Send fused position
```

### Step 3: Strategy updates (optional)
```python
# In strategy code when goals change:
if RERUN_AVAILABLE:
    update_target(goal_x, goal_y)
    update_trajectory(planned_path)
```

---

## 6. Visualizer Controls

| Control | Action |
|---------|--------|
| **Mouse drag** | Rotate 3D view |
| **Scroll** | Zoom in/out |
| **Right click drag** | Pan view |
| **Timeline slider** | Replay history |
| **Legend click** | Show/hide objects |
| **Right panel** | Time series plots |

---

## 7. Troubleshooting

### "Connection refused"
```bash
# Check if bridge is running:
ps aux | grep rerun_bridge

# Check port is open:
sudo lsof -i :9876

# Restart bridge:
pkill -f rerun_bridge
python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876
```

### "Position not updating"
```bash
# Verify odometry is being sent from Teensy:
python -c "
import sys; sys.path.insert(0, 'robot1/rasp')
from utils import init_robot
from loader import loader
Messages = loader.load_class('usb_com', 'Messages')
com, _ = init_robot(print)

def show_odom(data):
    import struct
    if len(data) >= 24:
        x, y, t = struct.unpack('<ddd', data[:24])
        print(f'Teensy odom: x={x:.1f} y={y:.1f} θ={t:.3f}')

com.add_callback(show_odom, Messages.UPDATE_ROLLING_BASIS.value)
import time
time.sleep(5)
"

# Should see messages like: "Teensy odom: x=1500.1 y=1000.2 θ=0.050"
```

### "LiDAR not showing"
```bash
# Check LiDAR thread is running:
python rerun_bridge.py --mode serve --with-lidar 2>&1 | grep -i lidar

# Should show: "✓ Lidar polling activé"

# If not, check LiDAR connection:
ls /dev/ttyUSB*
# Should show /dev/ttyUSB0 or /dev/ttyUSB1
```

### "High CPU usage"
```bash
# Reduce publishing rate:
# In rerun_bridge.py, change:
#   publish_loop(hz=20.0, ...)
# To:
#   publish_loop(hz=10.0, ...)  # 10 Hz instead

# Or disable LiDAR polling:
python rerun_bridge.py --mode serve --port 9876
# (without --with-lidar flag)
```

---

## 8. Common Tasks

### View LiDAR point cloud only
```bash
# On Rasp:
python rerun_bridge.py --mode local --with-lidar --sim

# On PC:
http://RaspIP:9876
# Enable only "world/lidar/cloud" in legend
```

### Record session for playback
```bash
# In Rerun viewer:
# 1. Click hamburger menu (☰)
# 2. "Save blueprint as..."
# 3. Viewer automatically records timeline

# Replay later:
# 1. Open saved blueprint
# 2. Drag timeline slider to rewind
```

### Export screenshot
```bash
# In Rerun viewer:
# 1. Adjust 3D view to desired angle
# 2. Right-click in 3D view
# 3. "Screenshot" (or use browser console)
```

---

## 9. Auto-Launch on Boot

### Option A: Systemd service (recommended)
```bash
# Create service file
sudo nano /etc/systemd/system/rerun-bridge.service
```

Paste:
```ini
[Unit]
Description=Rerun Bridge - Eurobot 2026
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/CDR-Nantes/robot1/rasp
Environment="PATH=/home/pi/.local/bin:/usr/bin"
ExecStart=/usr/bin/python3 /home/pi/CDR-Nantes/robot1/rasp/rerun/rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl enable rerun-bridge.service
sudo systemctl start rerun-bridge.service

# Check status:
sudo systemctl status rerun-bridge.service
```

### Option B: Cron (simpler)
```bash
crontab -e
# Add line:
@reboot /home/pi/start_rerun.sh
```

Create `/home/pi/start_rerun.sh`:
```bash
#!/bin/bash
cd /home/pi/CDR-Nantes/robot1/rasp/rerun
python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar >> /tmp/rerun.log 2>&1
```

Make executable:
```bash
chmod +x /home/pi/start_rerun.sh
```

---

## 10. What Gets Visualized

### 3D View (world/)
- 🔵 **Blue cylinder**: Teensy odometry position
- 🔴 **Red cylinder**: LiDAR calculated position (if localized)
- 🟢 **Green cylinder**: Fused best estimate
- 🟡 **Yellow point**: Navigation target
- ⚪ **Orange diamonds**: Detected beacons
- ⚪ **Gray cylinders**: Fixed beacons on field
- 📦 **Gray boxes**: Walls, obstacles
- 📊 **Dotted line**: Planned trajectory

### 2D View (sensors/lidar)
- 🟢 **Green points**: LiDAR rays
- 🔵 **Blue circles**: Reference grid (500mm spacing)

### Time Series (right panel)
- **Teensy**: X, Y, θ over time
- **LiDAR**: Confidence, # points, # beacons
- **Fused**: Best estimate X, Y
- **Fusion**: Odometry-to-LiDAR error in mm

---

## 11. Performance Expectations

| Metric | Value |
|--------|-------|
| Publishing rate | 20 Hz (50ms per frame) |
| Network bandwidth | ~500 KB/s |
| Typical latency | <100ms (LAN) |
| CPU overhead (Rasp) | ~15% |
| Memory overhead | ~50 MB |

---

## 12. Advanced: Custom Visualization

Add custom objects to 3D scene:

```python
# In rerun/rerun_bridge.py or custom code:
import rerun as rr

# Add a custom sphere
rr.log("world/custom/goal_area", rr.Boxes3D(
    centers=[[goal_x, goal_y, 0]],
    sizes=[[200, 200, 1]],  # 200mm radius circle
    colors=[[255, 200, 0, 100]],  # Transparent yellow
))

# Add a custom marker
rr.log("world/custom/current_target", rr.Points3D(
    positions=[[target_x, target_y, 100]],
    colors=[[255, 0, 0, 255]],  # Red
    radii=[50],
    labels=["TARGET"],
))
```

---

## 13. Next: Full Integration

For complete robot integration example, see:
- 📄 [RERUN_DATA_FLOW_INTEGRATION.md](RERUN_DATA_FLOW_INTEGRATION.md)
- 📄 [RERUN_INTEGRATION_EXAMPLES.md](RERUN_INTEGRATION_EXAMPLES.md)

---

## 14. Getting Help

**Check these sources:**

1. **Logs:** `sudo journalctl -u rerun-bridge.service -f`
2. **Documentation:** [RERUN_DATA_FLOW_INTEGRATION.md](RERUN_DATA_FLOW_INTEGRATION.md)
3. **Examples:** [RERUN_INTEGRATION_EXAMPLES.md](RERUN_INTEGRATION_EXAMPLES.md)
4. **Rerun Docs:** https://rerun.io/docs

---

## 15. Success Checklist

✅ **You know this is working when:**

1. Bridge starts without errors
2. Browser connects to `http://RaspIP:9876`
3. 3D terrain loads
4. Blue cylinder appears (Teensy position)
5. Blue cylinder moves when robot moves
6. LiDAR polar view updates (green rays)
7. Time series plots show smooth curves
8. Red cylinder shows when LiDAR localizes
9. Green cylinder blends the two estimates
10. Yellow point/line appear when goal is set

---

**Ready to integrate?** → [RERUN_INTEGRATION_EXAMPLES.md](RERUN_INTEGRATION_EXAMPLES.md)

