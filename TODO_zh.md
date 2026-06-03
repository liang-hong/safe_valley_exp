# Safe Valley 待办事项清单

## 已完成工作 ✅

### 核心功能
- [x] 实现基于flocking架构的safe_valley算法
- [x] 完成模块化的config、comm和main模块
- [x] 实现GPS时钟同步校准
- [x] 主程序模块化拆分，支持通过配置修改submode通道映射
- [x] 修改own_name读取逻辑兼容仿真与实机

### 状态机与导航
- [x] 规范submode起始状态记录
- [x] 修正formation移动逻辑
- [x] 修正leader导航轨迹计算逻辑
- [x] 修正leader_fix_origin订阅错误
- [x] 修改leader圆轨迹为相对当前位置向东10m的顺时针圆

### 仿真支持
- [x] 新增环境launch启动仿真环境和多机节点
- [x] 算法launch整合submode_publisher功能模拟RCIN切换submode
- [x] 新增launch合并启动mavros与算法程序，依赖wait_mavros脚本
- [x] 仿真cohe flock效果通过，align有效

### 实机支持
- [x] 修改RCin-submode三档开关边界值
- [x] 修改submode对应的RCchannel
- [x] 修改flock通信配置适应新集群（6 7 9 10，中东北南）
- [x] 修改安全半径（无人机半径）适配实机
- [x] 订阅leader的RCIn
- [x] 实机launch配置验证
- [x] sepa（分离）行为测试验证

### 脚本与工具
- [x] 更名flock_math为flock_method
- [x] 新增开机启动脚本
- [x] 给自启动脚本添加运行权限
- [x] 添加rosbag录制封装脚本
- [x] 添加rosbag录制的启动配置支持

### 文档
- [x] 新增中英文README文档
- [x] 添加中英文readme跳转链接
- [x] 更新中英文readme
- [x] 修正readme的实机操作说明
- [x] 修正子模式话题名称并新增rosbag录制文档

### 参数调整
- [x] 调整参数，恢复限速
- [x] 更新flock.yaml中的formation velocity参数
- [x] 更新target和leader heights参数

### 日志与调试
- [x] 为各执行模式添加启动位姿的日志输出
- [x] 修正仿真中的submode_publisher提示，修正无法退出的问题

---

## 正在进行的工作 ⏳

---

## 待完成工作 ⏳

### 核心功能
- [ ] 在无人机disarm（未解锁）状态下，通过遥控器指定的通道开关触发算法程序重置，使其回到统一坐标系初始化步骤并持续修正坐标基准，用于应对GPS-RTK定位切换等坐标突变的场景，允许手动重启算法进程以重新建立正确的坐标基准

### 测试与验证
- [ ] 坐标重建功能的测试验证

---

## 维护与改进
- [ ] 持续优化flocking参数配置
