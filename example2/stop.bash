#!/bin/bash

# Set default input element
if [ $# -eq 0 ]; then
  set -- "drone0"
fi

# Make a tmux list of sessions to be killed
tmux_session_list=("keyboard_teleop" "rosbag" "mocap" "gazebo" "drone0" "drone1" "drone2")

# For each drone namespace, add to the list
for ns in "$@"; do
  tmux_session_list+=("$ns")
done

# If inside tmux session, get the current session name
if [[ -n "$TMUX" ]]; then
  current_session=$(tmux display-message -p '#S')
fi

# Send Ctrl+C signal to each window of each session
for session in "${tmux_session_list[@]}"; do
  # Check if session exists
  if tmux has-session -t "$session" 2>/dev/null; then
    # Get the list of windows in the session
    windows=($(tmux list-windows -t "$session" -F "#{window_index}"))
    # Iterate through each window and send Ctrl+C
    for window in "${windows[@]}"; do
      # Send Ctrl+C to the window
      tmux send-keys -t "$session:$window" C-c
      sleep 0.1 # Add a small delay to allow the signal to be processed
    done
  fi
done

# # Kill gazebo
# pkill -9 -f "gazebo" < /dev/null

# # Kill gazebo bridges
# pkill -9 -f "ros_gz_bridge"

# Kill all tmux sessions from the list except for the current one
for session in "${tmux_session_list[@]}"; do
  if [[ "$session" != "$current_session" ]]; then
    tmux kill-session -t "$session" 2>/dev/null
  fi
done

# Kill the current tmux session, if in a tmux session
if [[ -n "$TMUX" ]]; then
  tmux kill-session -t "$current_session" 2>/dev/null
fi