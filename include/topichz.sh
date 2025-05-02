#!/bin/bash
for topic in $(rostopic list); do
    echo "=== $topic ==="
    rostopic hz -w 3 $topic &
    sleep 4
    pkill -f "rostopic hz -w 3 $topic"
done

