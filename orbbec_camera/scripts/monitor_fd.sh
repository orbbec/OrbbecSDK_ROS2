#!/bin/bash

executable_filename="component_container"

while true; do
  pids=$(pgrep -f "${executable_filename}")
  if [ -z "$pids" ]; then
    echo "The process ${executable_filename} is not running."
  else
    for pid in $pids; do
      fd_count=$(lsof -p "${pid}" 2>/dev/null | wc -l)
      echo "File descriptor count for process ${executable_filename} (${pid}): ${fd_count}"
    done
  fi
  sleep 1
done
