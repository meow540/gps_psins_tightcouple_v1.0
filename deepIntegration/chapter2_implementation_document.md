# 第二章实现文档（深组合：信号层检测与抑制）

## 1. 目标与边界

本章目标是在不新建 `realtime_deepCouple.m` 的前提下，基于现有原型完成“信号层检测 + 抑制”闭环：

1. 用 INS 预测多普勒与 GNSS 实测多普勒做一致性检测。
2. 检测到欺骗后，在跟踪层对 NCO 施加 INS 约束，降低伪信号牵引。
3. 在导航层对伪距/伪距率做修正，恢复可用导航解。

本章主文件仅限：

1. `deepIntegration/DeepCouple_perINStime.m`
2. `deepIntegration/perChannelTrackOnce_DeepIn.m`

可复用辅助模块：

1. `deepIntegration/postNavTight.m`
2. `deepIntegration/rhoSatRec_zcj.m`
3. `PVT/satpos_zcj.m`

## 2. 输入输出

### 2.1 输入

1. `trackResults`, `channel`, `eph`, `TOW`, `subFrameStart`
2. `ins*_trj.mat`（统一同源 INS 数据）
3. 场景：`clean`、`ds5`、`ds6`

### 2.2 输出

1. 历元状态：`normal/suspect/spoof`
2. 每星残差：`r_i[k]`
3. NCO 命令与辅助频率序列
4. 修正前后伪距/伪距率
5. 报警时刻与导航解

## 3. 统一数学定义（必须）

### 3.1 单位与符号

1. 频率：Hz
2. 速度：m/s
3. 距离：m
4. 时间：s
5. 实测多普勒：`f_meas = carrFreq - IF`

### 3.2 多普勒预测与几何投影算子

对卫星 `i`：

`f_ins,i = -(1/lambda) * e_i^T * (v_sat,i^e - v_rx,i^e) + f_satclk,i + f_rcvclk`

其中：

1. `lambda = c / f_L1`
2. `e_i` 为 ECEF 系 LOS 单位向量
3. `v_sat,i^e` 为卫星 ECEF 速度
4. `v_rx,i^e` 为接收机 ECEF 速度

几何投影算子要求：

1. 卫星速度与载体速度必须先统一到同一坐标系（推荐 ECEF）再投影到 LOS。
2. 若 INS 速度为导航系 `v_ins^n`，则先做 `v_ins^e = C_n2e * v_ins^n`。
3. 如使用杆臂补偿，天线速度采用  
   `v_ant^n = v_ins^n + C_b2n * (omega_ib_b x leverArm_b)`，  
   再转 ECEF：`v_ant^e = C_n2e * v_ant^n`。
4. 若无杆臂标定，本章显式采用 `leverArm_b = [0;0;0]`。

### 3.3 残差定义

`r_i = f_meas,i - f_ins,i`  
`r_i_c = r_i - b_i`（`b_i` 由 clean 标定）

### 3.4 钟速项闭环与单位统一（必须）

为闭合“钟速漂移”回路，载波参考频率统一为：

`f_NCO_ref,i = IF + f_INS_proj,i + f_clk_drift_hat`

注意单位：

1. EKF 钟速状态通常是距离变化率（m/s）。
2. 注入 NCO 前必须转换到 Hz：  
   `f_clk_drift_hat_Hz = (dot_rho_clk_hat_mps) / lambda`
3. 残差计算、NCO 注入、日志输出必须使用同一单位体系（Hz）。

实现要求：

1. 在 `DeepCouple_perINStime.m` 提取钟速状态并完成单位转换。
2. 在 `perChannelTrackOnce_DeepIn.m` 参与 `carrFreq` 合成。
3. 在残差计算中使用同一钟速定义，避免“钟漂公共偏置”误报。

## 4. 多速率同步（100Hz INS / 5Hz Nav / 1ms Tracking）

### 4.1 时间基准

1. 1ms：跟踪环推进（相干积分）
2. 100Hz：INS 机理化与状态传播
3. 5Hz：导航量测更新与检测决策

### 4.2 INS 辅助插值

在相邻 INS 辅助值 `f_aid_old` 与 `f_aid_new` 之间做 1ms 插值：

`f_aid(m) = f_aid_old + alpha_m * (f_aid_new - f_aid_old)`

### 4.3 相位连续性

必须保持 `remCarrPhase` 连续，禁止状态切换时出现载波相位跳变。

## 5. 欺骗检测机制

### 5.1 指标

1. 单星残差：`|r_i_c|`
2. 共模：`M_cm = |median_i(r_i_c)|`
3. 差模：`M_df = median_i |r_i_c - median_j(r_j_c)|`

### 5.2 clean 标定阈值

对 clean 窗口任意指标序列 `M[k]`：

1. `mu_M = median(M[k])`
2. `sigma_M = 1.4826 * median(|M[k] - mu_M|)`
3. `T_M = mu_M + k_M * sigma_M`

得到 `T_cm`, `T_df`, `T_sat`。

### 5.3 状态机

1. `normal`：常规跟踪
2. `suspect`：指标越界但未确认
3. `spoof`：连续 `K_confirm` 历元确认后进入并锁存

进入条件（至少其一）：

1. `M_cm > T_cm` 且 `N_hit >= N_minHit`
2. `M_df > T_df` 且 `N_hit >= N_minHit`

### 5.4 归一化创新阈值（增强版）

除固定阈值外，加入按通道预测协方差归一化门限：

`sigma_pred,i^2 = h_i * P * h_i' + sigma_meas,i^2`

`h_i` 由 LOS 与钟速项构造（对速度与钟速状态投影）。  
判据可写为：

`z_i = |r_i_c| / sqrt(sigma_pred,i^2)`，并与 `kappa` 比较。

实现要求：

1. 在文档和代码中固定 `P` 的状态索引映射（速度三维 + 钟速）。
2. `sigma_meas,i` 与跟踪环噪声模型一致。

## 6. 抑制机制（检测后）

### 6.1 载波环约束

统一载波频率命令：

`f_carr = IF + f_pll + w_aid * f_insAid + w_clk * f_clk_drift_hat`

`w_aid` 按状态分档：

1. `normal`：低
2. `suspect`：中
3. `spoof`：高

### 6.2 PLL 抑制策略

`spoof` 状态优先采用：

1. 降低 `pllNoiseBandwidth`（例如 2-3Hz）
2. 限制单步 `carrNco` 变化量
3. 必要时再限制鉴相器贡献（而不是一开始就硬置零）

### 6.3 码环辅助

保持载波辅助 DLL：

`codeFreq = codeFreqBasis - codeNco + (carrFreq - IF)/1540`

### 6.4 伪距/伪距率修正

1. `prr_corr,i = -lambda * f_insAid,i`
2. `rho_corr,i[k] = rho_corr,i[k-1] + prr_corr,i[k] * dt_nav`
3. 导航更新使用 `rho_corr/prr_corr`（替代或加权融合原始观测）

### 6.5 环路状态继承与切换平滑（必须）

状态切换时必须控制滤波器内部状态（`oldCarrNco`, `oldCarrError`）：

1. `normal -> spoof`：  
   - 将 `oldCarrNco` 对齐到当前 `f_clk_drift_hat` 与 `f_insAid` 对应的参考频率偏置  
   - 对 `oldCarrError` 进行冻结或限幅衰减，避免突变牵引
2. `spoof -> normal`（若实现恢复）：  
   - 基于当前 `remCarrPhase` 重初始化 PLL 连续状态  
   - 采用渐进权重恢复，不允许一步切回

目标：避免频率跳变、相位冲击和瞬时失锁。

## 7. EKF 与噪声模型要求

### 7.1 状态向量（建议最小集）

1. 位置误差 3
2. 速度误差 3
3. 姿态误差 3
4. 陀螺零偏 3
5. 加计零偏 3
6. 钟差、钟速 2

### 7.2 噪声分档

1. `normal`：标准 `Q/R`
2. `suspect`：增大 GNSS 相关观测噪声
3. `spoof`：以 INS 修正观测为主，显著抑制原始观测权重

## 8. 文件级任务清单

### 8.1 `deepIntegration/DeepCouple_perINStime.m`

新增：

1. clean 标定参数加载（`b_i`, `T_cm`, `T_df`, `T_sat`）
2. 残差与状态机
3. `f_clk_drift_hat` 提取、单位转换与下发
4. 抑制状态下参数分档控制
5. 伪距/伪距率修正链路
6. 关键日志与图形导出量

保留：

1. 现有多速率主骨架
2. 现有 INS/GNSS 推进流程

### 8.2 `deepIntegration/perChannelTrackOnce_DeepIn.m`

新增：

1. 统一 `f_carr` 合成（含 `w_aid`, `w_clk`）
2. `carrNco` 限幅
3. 状态切换下 `oldCarrNco/oldCarrError` 平滑处理
4. 调试输出：`f_meas`, `f_insAid`, `f_clk`, `f_cmd`

保留：

1. DLL/PLL 主流程
2. `remCarrPhase` 连续计算

## 9. 验收标准（ds5 + ds6）

必须同时满足：

1. clean 无误报
2. ds5 有报警并进入抑制状态
3. ds6 有报警并进入抑制状态
4. 抑制后导航优于未抑制对照
5. 多普勒对比图显示抑制后一致性提升

## 10. 输出内容

1. 状态时间线图（`normal/suspect/spoof`）
2. 四星多普勒对比图（`f_meas vs f_insAid`、`f_meas vs f_suppressed`）
3. 伪距/伪距率修正前后图
4. ENU 轨迹图（clean、spoof、抑制后）
5. 指标阈值图（`M_cm`, `M_df`, 可选 `z_i`）

## 11. 推荐实现顺序

1. 先打通时间同步、符号和单位一致性
2. 再做 clean 标定和固定阈值检测
3. 再接入归一化创新检测（`h_i P h_i' + R`）
4. 再接入 NCO 分档抑制和伪距修正
5. 最后做 ds5/ds6 全流程评估

## 12. 与第一章关系

1. 第一章：导航层检测 + INS 接管（`realtime_tightCouple.m`）
2. 第二章：信号层检测 + NCO/伪距修正（本文件，两原型实现）

两章独立，第二章不覆盖第一章结论。


## 13. 场景一致性约束（新增）
1. clean / ds5 / ds6 的真实运动轨迹应视为同一条参考轨迹；ds5 与 ds6 仅欺骗方式不同，不以“轨迹彼此分离”作为实现目标。
2. 第二章验收以“检测与抑制是否生效”为主：
   - clean 无误报；
   - ds5、ds6 能触发检测并进入抑制状态；
   - 抑制后导航解不劣化（或优于未抑制对照）。
3. ds5 与 ds6 允许差异的指标是：首次报警时刻、状态机停留时长、残差统计量、修正量大小；不要求轨迹形态出现显著差异。
4. 论文图中若轨迹高度重合属于预期现象，应通过多普勒残差、状态时间线、修正量曲线体现两场景差异。
