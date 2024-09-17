# Example 2: Plan execution control of a drone in Gazebo using aerostack2 behaviors

### 1. Launch Aerostack using tmuxinator

```bash
tmuxinator start -p aerostack2.yml
```

You can close tmux session using `stop.bash` script or manually killing tmux session (`tmux kill-session`).

### 2. Run mission
```bash
python3 mission.py
```