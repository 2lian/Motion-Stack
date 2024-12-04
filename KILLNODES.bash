#!/bin/bash
# set -e -o pipefail


humble
# Get the list of nodes
nodes=$(ros2 node list)
nodes="$nodes rviz_interface joint joint_node ik_heavy ik ik_node ik_heavy_node leg_node leg mover mover_node"

# Warning: nodes may have the same name, handle with care
echo "Killing the following nodes:"
echo "$nodes"

# Loop through each node and kill it
for node in $nodes; do
    # Skip 'rviz' and 'rviz2'
    if [[ "$node" == "/rviz" || "$node" == "/rviz2" ]]; then
        echo "Skipping node: $node"
        continue
    fi
    echo "Killing node: $node"
    pkill -f $node
done

echo "All nodes killed."
