# Robotics Lab — ROS 2 Strings Service + Twist + Central Agent Node

> **Goal (today):**  
> 1) Build and test a custom **ROS 2 service** (`/ask_llm`) using `strings_interfaces`  
> 2) Create a **Twist subscriber** that reads `/cmd_vel` velocity commands (robot driving)  
> 3) Create a **central node** that: takes user input → builds a smart prompt → calls the LLM service → publishes velocity commands

---

## Part 1 — Build + Run + Test the LLM String Service

### 1) Build the packages
From your ROS 2 workspace root:

```bash
colcon build --packages-select strings_interfaces
colcon build --packages-select llm_service_server
```

> If you build everything at once:
```bash
colcon build
```

### 2) Source the workspace
```bash
source install/setup.bash
```

### 4) Inspect the service + interface
In another terminal (Terminal B):

```bash
source install/setup.bash
ros2 service list
ros2 service info /ask_llm
ros2 interface show strings_interfaces/srv/StringService
```

### 5) Test the service call
```bash
ros2 service call /ask_llm strings_interfaces/srv/StringService "{request_data: 'What is the capital of France?, answer in one word'}"
```

---

## Part 2 — Twist Subscriber (Velocity Commands)

### What is `Twist`?
`Twist` is the standard ROS message for velocity commands:

- `linear` → translational velocity (**m/s**)
  - `linear.x` forward/back
  - `linear.y` left/right
  - `linear.z` up/down (usually unused for ground robots)

- `angular` → rotational velocity (**rad/s**)
  - `angular.z` yaw (turn left/right) ✅ most common
  - `angular.x`, `angular.y` (roll/pitch rates, usually unused)

For a ground robot you usually care about:
- `linear.x` (forward speed)
- `angular.z` (turning rate)

### Sample subscriber for `/cmd_vel`
Create a Python file (example: `twist_subscriber.py`) and use:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistSubscriber(Node):
    def __init__(self):
        super().__init__("twist_subscriber")
        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",          # topic name
            self.cb,             # callback
            10                   # queue size
        )
        self.get_logger().info("Listening to /cmd_vel (geometry_msgs/Twist)")

    def cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.get_logger().info(f"cmd_vel -> linear.x={vx:.3f}, linear.y={vy:.3f}, angular.z={wz:.3f}")


def main():
    rclpy.init()
    node = TwistSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Quick test (publish a Twist)
In a terminal:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" -r 5
```

Stop with `Ctrl+C`.

---

## Part 3 — Central “Brain” Node (User → LLM Service → cmd_vel)

### What you will build
A **central node** that does the following loop:

1. **Takes input** from the user (CLI input)  
2. Builds a **smart prompt** (large string)  
3. Acts as a **client** for the LLM service `/ask_llm`  
4. Parses the LLM response into a **driving command**  
5. Publishes a `Twist` to `/cmd_vel`
