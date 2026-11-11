## 虚拟下位机（ros2节点形式）

- 包名：`virtual_low_board`
- 节点名：`virtual_low_board_node`

- 话题：`/receive_packet`
- 类型：`std_msgs::msg::UInt8MultiArray`
- 包约定：见`packet.hpp`
- 可调参数：
  - `data_mode`: `smooth`/`random`，默认`smooth`

- 注：本包并没有遵循面向对象架构范式编程