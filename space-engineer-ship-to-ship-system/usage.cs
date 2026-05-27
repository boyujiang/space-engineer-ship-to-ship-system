class TrueThrustPID {
    // 完整的三项参数
    double kP = 2.0;   // 比例：看现在
    double kI = 0.1;   // 积分：看过去（历史累加）
    double kD = 0.6;   // 微分：看未来（速度阻尼）

    double errorSum = 0;   // 历史误差累加器（账本）
    double lastError = 0;
    const double dt = 1.0 / 60.0; // SE中Update1每帧固定的时间步长（秒）

    public float Control(double currentAngleError) {
        // 1.积分项：把这一帧的误差，累加到历史账本里
        errorSum += currentAngleError * dt;

        // 抗积分饱和（Anti-windup）：防止errorSum无限大导致导弹抽风，限制其最大累加能力
        errorSum = Math.Helper.Clamp(errorSum, -0.5, 0.5);

        // 2. 微分项：看这一帧和上一帧的变化率
        double errorDeriv = (currentAngleError - lastError) / dt;
        lastError = currentAngleError;

        // 3. 完整的 PID 公式
        double deceleration = (currentAngleError * kP) + (errorSum * kI) + (errorDeriv * kD);

        // 4. 转换成油门比例 (15% - 100%)
        double thrustOutput = 1.0 - deceleration;
        return (float)Math.Helper.Clamp(thrustOutput, 0.15, 1.0);
    }

    // 当导弹更换目标，或者重新发射时，必须清空历史账本
    public void Reset() {
        errorSum = 0;
        lastError = 0;
    }
}

public void Main(string argument, UpdateType updateSource)
{
    // ... 前面的冷发射（120帧）和 IGC 接收逻辑保持不变 ...

    if (SavedTargetPosition.LengthSquared() > 0.0001)
    {
        CalculateYaw_Pitch(SavedTargetPosition);
        AimTarget();

        // --- 【新增：推进器 PID 控制逻辑】 ---
        // 1. 计算当前导弹的总角度合成误差（利用勾股定理合成 Pitch 和 Yaw）
        double totalAngleError = Math.Sqrt(TargetPitchAngle * TargetPitchAngle + TargetYawAngle * TargetYawAngle);

        // 2. 将误差代入 PID，获取当前应该保持的推力百分比 (0.05f - 1.0f)
        float optimalThrustOverride = missileThrustController.Control(totalAngleError);

        // 3. 将计算出来的动态推力应用到所有向后的主推进器
        for (int i = 0; i < Thrusters.Count; i++) {
            if (Thrusters[i] == null) continue;
            
            // 注意：最好只控制向后喷的主推进器（可通过 WorldMatrix.Forward 筛选，这里默认你的 Thrusters 列表全为主推进器）
            Thrusters[i].Enabled = true;
            Thrusters[i].ThrustOverridePercentage = optimalThrustOverride;
        }

        // 显示调试信息到 LCD
        InfoLCD.WriteText($"角度误差: {(totalAngleError * (180/Math.PI)):F1}°\n", false);
        InfoLCD.WriteText($"当前油门: {(optimalThrustOverride * 100):F0}%\n", true);

        // 终末引爆
        if (RelativeCoordinate.Length() < ExplodeDistance)
        {
            for (int i = 0; i < Warheads.Count; i++) {
                if (Warheads[i] != null) Warheads[i].Detonate();
            }  
        }
    }
}

using System;
using System.Text.RegularExpressions; // 记得在脚本最上方引入正则命名空间

// ========================================================
// 核心全局变量声明
// ========================================================
int MissileID = -1;       // 默认值 -1，代表未识别或无编号盲射状态
bool IsLeader = false;     // 是否是大姐大

// 预编译正则表达式：匹配 "ID:" 后面跟着的任意多位数字（允许前后有空格）
// \b 代表单词边界，\d+ 代表匹配一个或多个数字
private static readonly Regex IdRegex = new Regex(@"\bID\s*:\s*(\d+)\b", RegexOptions.IgnoreCase);

void InitializeComponents()
{
    // 1. 基础组件获取逻辑（保留你原本的遍历）
    List<IMyTerminalBlock> allBlocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocks(allBlocks);
    // ... （此处省略主控和陀螺仪的获取，保持你原本的代码不变） ...

    if (MainCockpit == null) return;

    // 2. 【核心重构】使用 Regex 优雅解析 CustomData
    string rawData = Me.CustomData;
    Match match = IdRegex.Match(rawData);

    if (match.Success)
    {
        // match.Groups[1] 对应的是 (\d+) 括号里抓取到的纯数字文本
        if (int.TryParse(match.Groups[1].Value, out MissileID))
        {
            // 【Scalable 扩展核心】：不再用 hardcode 的 if-else 
            // 规则：只要母舰写给我的 ID 是 1，我就是首发大姐大
            if (MissileID == 1)
            {
                IsLeader = true;
            }
            
            Echo($"成功识别导弹编号! ID = {MissileID}, 战术定位: {(IsLeader ? "编队核心" : "僚机护航")}");
        }
    }
    else
    {
        // 容错机制：如果母舰脚本漏写、或者打印机出了故障 CustomData 为空
        MissileID = 999; 
        IsLeader = false;
        Echo("警告: CustomData 未匹配到有效ID，切换为默认后备编号");
    }

    // 3. 接下来走你原本的陀螺仪矩阵方向碰撞表映射逻辑（构建 gyroYawField 等）...
    // BuildGyroMatrixMapping(); 
}