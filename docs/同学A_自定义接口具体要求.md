# 同学 A：自定义接口具体要求

> 依据：[ROS2 双人巡检项目总需求](../ROS2_双人巡检项目需求.md)第 3、4、6 节。本文件只写接口要求，不创建 `.msg`、`.srv`、`.action` 源文件。下面的字段类型和辅助消息是**可直接照着实现的第一版建议**；合并前须与 B 同学确认，因为 `inspection_interfaces` 是双方共用的包。

## 1. 你到底要设计哪些接口

| 类别 | 你负责定义并使用 | 你需要与 B 对接，但由 B 负责业务实现 |
|---|---|---|
| `.msg` | `TargetStation`、`ArrivalEvent`、`MissionStatus`、`MissionStatusArray`、`QueueEntry`、`QueueStatus`、`RobotOccupancy` | `HealthState`、`InspectionReport` |
| `.srv` | `GetStation`、`SetMissionPause`、`GetMissionState`、`GetMissionQueue` | `SetInspectionPause`、`GetInspectionReport` |
| `.action` | `InspectRoute`：A 做服务端 | `CollectStation`：A 做客户端，B 做服务端 |

这些自定义类型都放在共同的 `src/inspection_interfaces/` 包中，按 `msg/`、`srv/`、`action/` 分目录。**接口类型的定义由双方共同评审；节点实现按 A/B 职责分工。**

## 2. 先看懂文件格式

`.msg` 每行是一个字段：

```text
string task_id
uint8 priority
```

`.srv` 用 `---` 分开“请求”和“响应”：

```text
string station_id
---
bool found
string name
```

`.action` 必须按 **Goal（请求）→ Result（最终结果）→ Feedback（过程反馈）** 排列，用两个 `---` 分开。总需求中的表格先列 Feedback 再列 Result，是为了阅读方便；写文件时仍按 ROS 2 的格式。

下文代码块是**建议写入文件的具体内容**，你和 B 审查通过后再创建对应文件。

## 3. 双方先约定的通用规则

- `robot_id` 第一版只允许 `r1`、`r2`，Topic 路径和消息体的 `robot_id` 必须一致。
- `task_id` 由提交任务的人生成；同一 ID 不得指向两条不同路线。`station_id` 必须存在于共同 YAML 中，大小写敏感。
- 时间字段使用 `builtin_interfaces/Time`。`deadline` 全零表示“没有截止时间”。超时计时使用节点收到事件时的时钟，不能只比较消息自带时间。
- 站点索引从 **0** 开始；队列位置从 **1** 开始。没有当前站点或不在队列时用 `-1`。
- `error_code` 用 `uint16`，`0` 表示无错误；失败时还要填写人能看懂的 `detail`。
- 目标、状态、队列快照建议 `reliable + transient local`；到站事件建议 `reliable + volatile`，避免重启后旧到站消息被重放。双方实际发布/订阅 QoS 必须兼容。
- `MissionStatusArray` 和 `QueueStatus` 是**完整快照**，列表为空时也要发布空数组，避免观察者误以为旧任务仍在运行。

## 4. 你负责的 `.msg`

### 4.1 `TargetStation.msg`：发给机器人身份的目标

Topic：A 发布到 `/robots/<robot_id>/target_station`。

```text
string robot_id
string task_id
string station_id
int32 station_index
uint32 station_count
bool active
builtin_interfaces/Time stamp
```

**要求**：`active=true` 表示执行该目标；取消、结束或故障接管时，先对旧机器人发布同一任务/站点的 `active=false`。`station_index` 从 0 开始。`stamp` 是发布时间。`r1` 和 `r2` 使用各自 Topic，不可串线。

### 4.2 `ArrivalEvent.msg`：接收外部到站事件

Topic：A 订阅 `/robots/<robot_id>/arrival`。

```text
string robot_id
string task_id
string station_id
builtin_interfaces/Time arrived_at
```

**要求**：只有 Topic 命名空间、`robot_id`、`task_id`、当前 `station_id` 全部匹配时才能推进任务。错机器人、错任务、错站点、重复或过期到站均忽略并记日志。到站消息由命令或测试程序提供，本项目不开发机器人运动节点。

### 4.3 `MissionStatus.msg`：一条任务的当前状态

这是 `MissionStatusArray` 和查询服务共用的辅助消息。

```text
uint8 UNKNOWN=0
uint8 QUEUED=1
uint8 WAITING_FOR_ROBOT=2
uint8 WAITING_ARRIVAL=3
uint8 COLLECTING=4
uint8 PAUSED_MANUAL=5
uint8 PAUSED_HEALTH=6
uint8 PAUSED_DEPENDENCY=7
uint8 RECOVERY_REQUIRED=8
uint8 SUCCEEDED=9
uint8 CANCELED=10
uint8 FAILED=11

string task_id
string robot_id
uint8 state
int32 queue_position
string current_station_id
int32 current_index
uint32 completed_count
bool paused_manual
bool paused_health
bool paused_dependency
uint16 error_code
string detail
builtin_interfaces/Time stamp
```

**要求**：`state` 使用文件顶部常量；无机器人时 `robot_id` 为空，无当前站点时 `current_station_id` 为空且 `current_index=-1`；不在队列时 `queue_position=-1`。三种暂停原因可能同时存在，因此除了主状态还要有三个布尔字段；只清除其中一种原因不能让任务继续。状态变化时更新时间戳和详情。

### 4.4 `MissionStatusArray.msg`：全部未结束任务的快照

Topic：A 发布到 `/mission/status`。

```text
builtin_interfaces/Time stamp
MissionStatus[] missions
```

**要求**：包含所有尚未结束的任务，不只包含正在运行的任务。队列位置、机器人分配、当前站点或状态一有变化就重新发布；启动后先发布空快照。已经结束的任务不在此数组中，但查询服务应能给出其最终状态。

### 4.5 `QueueEntry.msg` 与 `QueueStatus.msg`：队列项和队列快照

`QueueEntry.msg`：

```text
string task_id
uint8 priority
builtin_interfaces/Time deadline
int32 position
```

`QueueStatus.msg`：

```text
builtin_interfaces/Time stamp
QueueEntry[] pending_tasks
```

Topic：A 发布 `QueueStatus` 到 `/mission/queue`。`pending_tasks` 按实际调度顺序排列，`position` 从 1 开始；队列空时发布空数组。排序规则是：优先级高的在前；同优先级先看更早的非零截止时间；再看入队时间；最后看 `task_id`。零截止时间排在有截止时间之后。

### 4.6 `RobotOccupancy.msg`：查询两台机器人的占用情况

给 `GetMissionQueue.srv` 使用：

```text
string robot_id
bool occupied
string task_id
```

响应中固定给出 `r1` 和 `r2` 两项。空闲时 `occupied=false`、`task_id` 为空。这个消息只表明**是否被任务占用**；机器人是否健康由 B 的 `HealthState` 表示。

## 5. 你负责的 `.srv`

### 5.1 `GetStation.srv`：查询站点

服务名：`/stations/get`。

```text
string station_id
---
bool found
string name
string description
```

站点数据从共同 YAML 读取。未知 ID 返回 `found=false`；不要让服务超时或抛异常。`name` 是显示名称，`description` 是简短说明。

### 5.2 `SetMissionPause.srv`：暂停或继续一条任务

服务名：`/mission/set_pause`。

```text
string task_id
bool pause
---
bool accepted
MissionStatus state
string detail
```

`pause=true` 只添加**人工暂停**原因；`pause=false` 只解除人工暂停。若机器人仍有健康故障或 B 仍不可用，任务必须继续暂停。采集中暂停时，A 还要调用 B 的 `/inspection/set_pause`。未知或已结束任务返回 `accepted=false` 并说明原因。

### 5.3 `GetMissionState.srv`：查询一条任务

服务名：`/mission/get_state`。

```text
string task_id
---
bool found
MissionStatus state
string detail
```

返回任务状态、机器人、当前站点、队列位置、错误码等。未知任务返回 `found=false` 和原因，调用方不能把默认的 `state` 字段当成有效状态。

### 5.4 `GetMissionQueue.srv`：查询整个队列

服务名：`/mission/get_queue`。

```text
---
QueueEntry[] pending_tasks
RobotOccupancy[] robots
```

没有请求字段。`pending_tasks` 必须与 `/mission/queue` 的最新顺序一致；`robots` 固定包含 `r1`、`r2` 的占用状态。无任务时仍正常返回空队列和两项空闲机器人。

## 6. 你负责的 `.action`

### `InspectRoute.action`：整条路线的长任务

Action 名：`/mission/inspect_route`。用户/测试程序是客户端，A 的 `mission_manager` 是服务端。

```text
string task_id
string[] station_ids
uint8 priority
builtin_interfaces/Time deadline
bool resume_existing
---
bool success
uint16 error_code
string detail
InspectionReport[] reports
uint32 reassignment_count
---
uint8 state
string robot_id
int32 queue_position
string current_station_id
int32 current_index
uint32 completed_count
uint32 reassignment_count
bool paused_manual
bool paused_health
bool paused_dependency
string detail
```

**Goal 校验**：`task_id` 非空且唯一；`station_ids` 非空、无重复、全部存在于站点配置中，最多 10 站；`priority` 为 0～9；非零截止时间不能已经过期。最多两条活动任务、八条排队任务；队列满或 ID 冲突时拒绝新 Goal。

**Feedback**：排队、分配、进入新站点、接管、等待和状态变化时发送；`state` 使用 `MissionStatus.msg` 的常量。`queue_position=-1` 表示不在队列。需要定期反馈时按参数 `feedback_period_sec`。

**Result**：保留所有已完成站点的 `InspectionReport`；成功、失败、取消分别设置 ROS 2 Action 对应终态。取消活动任务时要取消 B 的子 Action、发布旧目标 `active=false`、释放机器人，不能重复产生终态。

**恢复**：`resume_existing=true` 表示用户尝试恢复原任务；除这个开关外，任务 ID、站点列表、优先级、截止时间必须与保存的原任务一致。恢复实现安排在第四周。

## 7. 你必须与 B 对齐的接口

这些文件由双方共同评审，B 实现服务端/发布端，A 必须能正确读取或调用：

| B 的接口 | A 用来做什么 | 最少要确认的字段或语义 |
|---|---|---|
| `HealthState.msg`，`/robots/<id>/health/state` | 判断哪台机器人健康、是否可分配 | `robot_id`、健康等级、电量百分比、`battery_ok`、`heartbeat_ok`、告警是否锁存、`detail`、时间戳。尚未收到有效电量和心跳时为 `UNKNOWN`。 |
| `InspectionReport.msg` | 放进总任务的 `reports[]`，避免重复报告 | `robot_id`、`task_id`、`station_id`、样本数、平均温度、最大振动、`passed`、`error_code`、`detail`、完成时间。以任务 ID＋站点 ID 去重。 |
| `CollectStation.action`，`/inspection/collect_station` | A 在匹配到站后请求 B 采集 | Goal：机器人/任务/站点 ID；Feedback：有效/拒绝/目标样本数、耗时、详情；Result：`success`、错误码、详情、报告。 |
| `SetInspectionPause.srv` | A 暂停或继续正在采集的站点 | 请求：任务 ID、站点 ID、`pause`；响应：是否接受和原因。重复调用要安全。 |
| `GetInspectionReport.srv` | A 重启恢复时查询已完成报告 | 请求：任务 ID、站点 ID；响应：是否找到、报告、原因。 |
| `/inspection/availability` | B 停用或不可用时 A 暂停依赖采集的任务 | 使用现成的 `std_msgs/msg/Bool`，不需要自定义 `.msg`。 |

**特别注意**：B 的采集 Action `success=true` 表示采集流程完成，`report.passed=false` 仍可能表示数据不合格。A 按 `allow_skip_failed_station` 决定是否继续；不能把这两个布尔值混为一谈。

## 8. A 的错误码与第一周交付

| 错误码 | A 应在什么情况下使用 |
|---:|---|
| `0` | 无错误 |
| `101` | Goal 无效、未知站点等 |
| `102` | 队列已满或任务 ID 冲突 |
| `103` | 到站或机器人恢复超时 |
| `104` | 子 Action 不可用 |
| `105` | 截止时间已过 |
| `106` | 接管次数超过上限 |
| `107` | 恢复请求与保存的原任务不一致 |
| `108` | B 的采集节点不可用或停用 |

**第一周应完成**：双方审查并定稿全部自定义接口；让共同的接口包构建成功；A 实现站点/队列查询、状态与队列快照、目标发布能力、生命周期节点骨架，并写出状态转移表。`InspectRoute` 的多 Goal 执行在第二周，接管和暂停联动在第三周，持久化恢复在第四周。

### 交付前逐项检查

- [ ] B 已确认 `InspectionReport`、`HealthState`、`CollectStation` 与两个 B 服务的字段。
- [ ] `.msg`、`.srv`、`.action` 的文件名、类型、Topic/Service/Action 名称一致。
- [ ] `ros2 interface show` 能显示双方合并后的定义；接口包可在干净工作区构建。
- [ ] `r1` 与 `r2` 的命名空间不串线；到站要同时匹配机器人、任务、站点。
- [ ] 任务和队列的空快照、晚启动订阅者、未知站点查询都可验证。
- [ ] 修改任何共同接口时，同步更新 A/B 调用方与文档。
