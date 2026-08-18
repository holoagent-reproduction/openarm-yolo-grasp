# OpenArm Semantic Pick Skill

该 Skill 模仿 HoloAgent 的高层技能接口，但只连接本项目的 `SemanticPick` Action，不依赖或修改 HoloAgent 仓库。

启动 ROS 侧 Skill 适配器后，可通过脚本提交、查询、确认和取消任务。默认地址为 `http://NUC地址:8780`。

真实执行必须同时满足：标定配置通过、启动参数 `allow_motion=true`、MoveIt 控制器正常、操作者人工确认。
