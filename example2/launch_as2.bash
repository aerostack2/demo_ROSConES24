#!/bin/bash

usage() {
    echo "  options:"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "t" opt; do
  case ${opt} in
    t )
      launch_keyboard_teleop="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

simulation_config="world.json"
drone="drone0"

tmuxinator start -p aerostack2.yml \
    drone_namespace=${drone} \
    simulation_config=${simulation_config} \
    keyboard_teleop=${launch_keyboard_teleop} &
wait

# Attach to tmux session ${drone_arr[@]}, window mission
tmux attach-session -t drone0:mission
