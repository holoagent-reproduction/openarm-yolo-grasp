---
name: semantic-pick-skill
description: 通过 OpenArm 的受监控 SemanticPick 接口识别并拿起桌面上的指定物品。真实执行必须等待人工确认。
---

# Semantic Pick Skill

## 适用场景

用户明确要求拿起水杯、马克杯、塑料瓶、易拉罐、纸盒或小盒子时使用。

## 调用规则

1. 调用 `POST /skills/semantic_pick`，提交原始中文指令和 `preview_only`。
2. 轮询任务状态，直到 `wait_confirm`、`succeeded` 或 `failed`。
3. `wait_confirm` 状态下不得自行确认；必须等待操作者检查检测图、三维位姿和规划轨迹。
4. 只有操作者明确确认后，调用任务的 `confirm` 接口。
5. 任意失败都原样报告错误码，不自动重试真实抓取。

## 安全限制

- 不发布关节命令。
- 不绕过标定、工作区、碰撞检查和人工确认门控。
- 不支持透明、柔软、线状或严重反光物体。
