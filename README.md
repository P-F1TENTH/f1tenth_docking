# F1/10 Docking
This package is used to dock the F1/10 car to the docking station.

```mermaid
flowchart LR

/docking_action_server[ /docking_action_server ]:::main

/optitrack/rigid_body_0([ /optitrack/rigid_body_0<br>geometry_msgs/msg/PoseStamped ]):::bugged
/vesc/core([ /vesc/core<br>vesc_msgs/msg/VescStateStamped ]):::bugged
/commands/ctrl([ /commands/ctrl<br>control_interfaces/msg/Control ]):::bugged
/mpc/predicted_states([ /mpc/predicted_states<br>geometry_msgs/msg/PoseArray ]):::bugged

/docking_action_server{{ /docking_action_server<br>f1tenth_docking_interfaces/action/Docking }}:::bugged
/optitrack/rigid_body_0 --> /docking_action_server
/vesc/core --> /docking_action_server
/docking_action_server --> /commands/ctrl
/docking_action_server --> /mpc/predicted_states


/docking_action_server o==o /docking_action_server

subgraph keys[<b>Keys<b/>]
subgraph nodes[<b><b/>]
topicb((No connected)):::bugged
main_node[main]:::main
end
subgraph connection[<b><b/>]
node1[node1]:::node
node2[node2]:::node
node1 o-.-o|to server| service[/Service<br>service/Type\]:::service
service <-.->|to client| node2
node1 -->|publish| topic([Topic<br>topic/Type]):::topic
topic -->|subscribe| node2
node1 o==o|to server| action{{/Action<br>action/Type/}}:::action
action <==>|to client| node2
end
end
classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
style keys opacity:0.15,fill:#FFF
style nodes opacity:0.15,fill:#FFF
style connection opacity:0.15,fill:#FFF
linkStyle 4,9 fill:none,stroke:green;
```


### How to test

Edit the config file:
```bash
nvim f1tenth_docking/config/config.yaml
```

Start the docking action server:
```bash
ros2 launch f1tenth_docking f1tenth_docking.launch.py
```

Mock the vesc servo pose:
```bash
ros2 topic pub -r 100 /vesc/core vesc_msgs/VescStateStamped '{state: {servo_pose: 0.0}}'
```

Mock the optitrack pose:
```bash
ros2 topic pub -r 100 /optitrack/rigid_body_0 geometry_msgs/PoseStamped '{pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

Set the control loop setpoint:
```bash
ros2 action send_goal docking_action_server f1tenth_docking_interfaces/action/Docking "{setpoint: {x_pos: 497, y_pos: 2.0, theta: 0.0, delta: 0.0}}"
```
