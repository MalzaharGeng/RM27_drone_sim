# EXCITE 历史回放输入

两份 CSV 分别来自 FIX18 正常轮和中断前失败轮。只截取全部 EXCITE 行及紧随其后的第一条判定行，字段值未修改；来源与完整原文件 SHA256 见 [provenance.json](provenance.json)。

它们用于复现历史 reset 与真实生产 `Pilot.bounded_excite_decision()` 的决策，不是新飞行数据。测试不会构造 ROS 节点或连接 PX4。
