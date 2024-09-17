#!/bin/bash

# Make a tmux list of sessions to be killed
tmux_session_list=("drone0")

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

# Kill all tmux sessions from the list except for the current one
for session in "${tmux_session_list[@]}"; do
  if [[ "$session" != "$current_session" ]]; then
    tmux kill-session -t "$session" 2>/dev/null
  fi
done

# Because sometimes it doesn't close well
# Kill gazebo
pkill -9 -f "gazebo" < /dev/null

# Kill gazebo bridges
pkill -9 -f "ros_gz_bridge"

# Kill the current tmux session, if in a tmux session
if [[ -n "$TMUX" ]]; then
  tmux kill-session -t "$current_session" 2>/dev/null
fi