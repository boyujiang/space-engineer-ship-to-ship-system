using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
public class Program : MyGridProgram
{
const string Radiotag = "WenYue";

    Double ExplodeDistance = 1;
    // ========================================================
    // 1. PID 参数配置配置区 (根据导弹手感微调这几个数即可)
    // ========================================================
    // P(比例): 决定转头有多快; D(微分): 专门管住 Overshoot 刹车的阻尼; I(积分): 消除最后微小静差
    
    // 俯仰轴 (Pitch/上下) PID 参数
    public const double P_Pitch = 4.5;
    public const double I_Pitch = 0.1;
    public const double D_Pitch = 1.8;

    // 偏航轴 (Yaw/左右) PID 参数
    public const double P_Yaw = 4.5;
    public const double I_Yaw = 0.1;
    public const double D_Yaw = 1.8;

    // 翻滚轴 (Roll/自旋) PID 参数 (通常弹体窄, 阻尼可以小点)
    public const double P_Roll = 3.0;
    public const double I_Roll = 0.1;
    public const double D_Roll = 1.0;

    //Thruster PID参数
    public const double P_Thrust = 2.0;
    public const double I_Thrust = 0.1;
    public const double D_Thrust = 0.5;

    // 积分限幅：防止大角度转弯时积分盲目累加导致过冲
    public const double Max_I_Windup = 0.5; 

    // ========================================================
    // 2. 系统核心组件与 PID 实例
    // ========================================================
    IMyShipController MainCockpit; // 导弹主控核心 (提供其 Forward/Up/Left 局部坐标系)

    IMyRadioAntenna Antenna;
    IMyBroadcastListener MissileListener; // 【修复】现代 IGC 监听器全局存储

    IMyShipConnector Connector;

    IMyGasTank Tank;
    List<IMyGyro> GyroList = new List<IMyGyro>();

    List<IMyWarhead> WarheadList = new List<IMyWarhead>();

    List<IMyThrust> ThrustList = new List<IMyThrust>();

    // 声明三轴独立的 PID 控制器
    PIDController PitchPID = new PIDController(P_Pitch, I_Pitch, D_Pitch, Max_I_Windup);
    PIDController YawPID = new PIDController(P_Yaw, I_Yaw, D_Yaw, Max_I_Windup);
    PIDController RollPID = new PIDController(P_Roll, I_Roll, D_Roll, Max_I_Windup);

    PIDController ThrustPID = new PIDController(P_Thrust, I_Thrust, D_Thrust, Max_I_Windup);

    // ========================================================
    // 3. 陀螺仪多轴向动态映射表 (完整保留你的核心灵魂)
    // ========================================================
    List<string> gyroYawField = new List<string>();
    List<float> gyroYawFactor = new List<float>();

    List<string> gyroPitchField = new List<string>();
    List<float> gyroPitchFactor = new List<float>();

    List<string> gyroRollField = new List<string>();
    List<float> gyroRollFactor = new List<float>();

    Vector3D SavedTargetPosition = new Vector3D();

    bool IsInitialized=false;
    long count = 0;
    Vector3D RelativeCoordinate;

    public Program()
    {
        // 每帧执行 (1/60秒)，保证 PID 微分项获取高频连续的误差变化率
        Runtime.UpdateFrequency = UpdateFrequency.Update1;
        InitializeComponents();
    }

    public void Main(string argument, UpdateType updateSource)
    {

        if (!IsInitialized) {
            InitializeComponents();
            return;
        }
        if (MainCockpit == null || GyroList.Count == 0)
        {
            Echo("错误: 缺少主控或陀螺仪！");
            return;
        }
        if (count <1) {   
            for (int i=0;i<WarheadList.Count;i++)
                {
                    
                }
            for (int i = 0; i < ThrustList.Count; i++) {
                if (ThrustList[i] == null) continue;
                ThrustList[i].Enabled = true;
                ThrustList[i].ThrustOverridePercentage = 1f;

            }  
            if (Connector.IsConnected){Connector.Disconnect();}
            Connector.Enabled=false;
            if(Tank.Stockpile){Tank.Stockpile=false;}
        }
        count++;
        if (count < 90) return;

        // 获取目标空间坐标 (假设无线电拿到了 SavedTargetPosition)
        GetTarget();

        //矩阵变换：将目标世界坐标转换为导弹主控的局部坐标 (X=方位, Y=高度, Z=深度)
        //
        if (SavedTargetPosition.LengthSquared() > 0.0001){
            //MatrixD lookAt = MatrixD.CreateLookAt(Vector3D.Zero, MainCockpit.WorldMatrix.Forward, MainCockpit.WorldMatrix.Up);
            //Vector3D TargetRelativePos = Vector3D.TransformNormal(SavedTargetPosition - MainCockpit.GetPosition(), lookAt);
            Vector3D TargetRelativePos = Vector3D.TransformNormal(SavedTargetPosition - MainCockpit.GetPosition(), MatrixD.Transpose(MainCockpit.WorldMatrix));
            //MainCockpit.WorldMatrix本身带translation但这里默认TransformNormal计算时没有用到
            //计算误差角度 (通过弧度直接作为 PID 输入)
            double distanceDepth = TargetRelativePos.Z;
            //if (distanceDepth < 0.1) distanceDepth = 0.1; // 防止除以 0

            // 核心解算：用你推导的 Azimuth 和 Height 算出当前的绝对角度误差
            double errorYaw = Math.Atan2(TargetRelativePos.X, -distanceDepth);   // 目标偏航误差
            double H_dist=Math.Sqrt(TargetRelativePos.X * TargetRelativePos.X + distanceDepth * distanceDepth);
            double errorPitch = Math.Atan2(TargetRelativePos.Y, H_dist); 
            
            // 强制要求导弹在飞行中不自旋，锁定 Roll 角度误差为 0 
            double errorRoll = 0; 

            // PID 控制器动态解算：吃进角度误差，吐出期望的物理角速度 (Desired Angular Velocity)
            // 这一步彻底干掉了你以前算 stopTime、加速度的暴力公式，改用微分阻尼柔性刹车
            float desiredYawVelocity = (float)YawPID.Control(errorYaw);
            float desiredPitchVelocity = (float)PitchPID.Control(errorPitch);
            float desiredRollVelocity = (float)RollPID.Control(errorRoll);

            //根据角度偏差调整thruster功率
            double totalAngleError = Math.Sqrt(errorPitch * errorPitch + errorYaw * errorYaw);
            float deceleration = (float)ThrustPID.Control(totalAngleError);
            double thrustOutput = 1.0 - deceleration;
            for (int i = 0; i < ThrustList.Count; i++) {
                if (ThrustList[i] == null) continue;
                
                // 注意：最好只控制向后喷的主推进器（可通过 WorldMatrix.Forward 筛选，这里默认你的 Thrusters 列表全为主推进器）
                ThrustList[i].Enabled = true;
                ThrustList[i].ThrustOverridePercentage = (float)VRageMath.MathHelper.Clamp(thrustOutput,0.01, 1.0);
            }

            // 完美的真理传导：利用你当年写的映射表，把期望角速度送进不同朝向的陀螺仪
            for (int i = 0; i < GyroList.Count; i++)
            {
                IMyGyro gyro = GyroList[i];
                gyro.GyroOverride = true;

                // 清空上一帧的物理值
                gyro.Pitch = 0;
                gyro.Yaw = 0;
                gyro.Roll = 0;

                // 映射 Yaw 期望输出
                gyro.SetValueFloat(gyroYawField[i], desiredYawVelocity * gyroYawFactor[i]);
                // 映射 Pitch 期望输出
                gyro.SetValueFloat(gyroPitchField[i], desiredPitchVelocity * gyroPitchFactor[i]);
                // 映射 Roll 期望输出
                gyro.SetValueFloat(gyroRollField[i], desiredRollVelocity * gyroRollFactor[i]);
            }
            if (TargetRelativePos.Length() < ExplodeDistance)
            {
                for (int i = 0; i < WarheadList.Count; i++) {
                    if (WarheadList[i] != null) WarheadList[i].Detonate();
                }  
            }
        }
    }

    // ========================================================
    // 4. 初始化与空间网格映射逻辑
    // ========================================================
    void InitializeComponents()
    {
        List<IMyTerminalBlock> allBlocks = new List<IMyTerminalBlock>();
    
        // 不管是用合并块还是连接器，只抓取属于这个编程块（Me）自身网格的方块
        GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(allBlocks, b => b.CubeGrid == Me.CubeGrid);

        foreach (var block in allBlocks)
        {
            if (block is IMyShipController && block.IsFunctional)
            {
                MainCockpit = (IMyShipController)block;
            }
            if (block is IMyRadioAntenna && block.IsFunctional)
            {
                Antenna = (IMyRadioAntenna)block;
            }
            if (block is IMyShipConnector && block.IsFunctional)
            {
                Connector = (IMyShipConnector)block;
            }
            if (block is IMyGasTank && block.IsFunctional)
            {
                Tank = (IMyGasTank)block;
            }
            if (block is IMyGyro && block.IsFunctional)
            {
                GyroList.Add((IMyGyro)block);
            }
            if (block is IMyWarhead && block.IsFunctional)
            {
                WarheadList.Add((IMyWarhead)block);
            }
            if (block is IMyThrust && block.IsFunctional)
            {
                ThrustList.Add((IMyThrust)block);
            }
        }

        if (MainCockpit == null) return;

        // 遍历所有陀螺仪，构建局部坐标到全局运动的矩阵碰撞映射
        for (int i = 0; i < GyroList.Count; i++)
        {

            // 检查陀螺仪的 Local Up 正对主控的哪个面
            Base6Directions.Direction gyroUp = GyroList[i].WorldMatrix.GetClosestDirection(MainCockpit.WorldMatrix.Up);  

            Base6Directions.Direction gyroLeft = GyroList[i].WorldMatrix.GetClosestDirection(MainCockpit.WorldMatrix.Left);  

            Base6Directions.Direction gyroForward = GyroList[i].WorldMatrix.GetClosestDirection(MainCockpit.WorldMatrix.Forward);  
            switch (gyroUp)
            {
                case Base6Directions.Direction.Up: gyroYawField.Add("Yaw"); gyroYawFactor.Add(1f); break;
                case Base6Directions.Direction.Down: gyroYawField.Add("Yaw"); gyroYawFactor.Add(-1f); break;
                case Base6Directions.Direction.Left: gyroYawField.Add("Pitch"); gyroYawFactor.Add(1f); break;
                case Base6Directions.Direction.Right: gyroYawField.Add("Pitch"); gyroYawFactor.Add(-1f); break;
                case Base6Directions.Direction.Forward: gyroYawField.Add("Roll"); gyroYawFactor.Add(-1f); break;
                case Base6Directions.Direction.Backward: gyroYawField.Add("Roll"); gyroYawFactor.Add(1f); break;
            }

            // 检查陀螺仪的 Local Left 正对主控的哪个面
            
            switch (gyroLeft)
            {
                case Base6Directions.Direction.Up: gyroPitchField.Add("Yaw"); gyroPitchFactor.Add(1f); break;
                case Base6Directions.Direction.Down: gyroPitchField.Add("Yaw"); gyroPitchFactor.Add(-1f); break;
                case Base6Directions.Direction.Left: gyroPitchField.Add("Pitch"); gyroPitchFactor.Add(1f); break;
                case Base6Directions.Direction.Right: gyroPitchField.Add("Pitch"); gyroPitchFactor.Add(-1f); break;
                case Base6Directions.Direction.Forward: gyroPitchField.Add("Roll"); gyroPitchFactor.Add(-1f); break;
                case Base6Directions.Direction.Backward: gyroPitchField.Add("Roll"); gyroPitchFactor.Add(1f); break;
            }

            // 检查陀螺仪的 Local Forward 正对主控的哪个面
            
            switch (gyroForward)
            {
                case Base6Directions.Direction.Up: gyroRollField.Add("Yaw"); gyroRollFactor.Add(1f); break;
                case Base6Directions.Direction.Down: gyroRollField.Add("Yaw"); gyroRollFactor.Add(-1f); break;
                case Base6Directions.Direction.Left: gyroRollField.Add("Pitch"); gyroRollFactor.Add(1f); break;
                case Base6Directions.Direction.Right: gyroRollField.Add("Pitch"); gyroRollFactor.Add(-1f); break;
                case Base6Directions.Direction.Forward: gyroRollField.Add("Roll"); gyroRollFactor.Add(-1f); break;
                case Base6Directions.Direction.Backward: gyroRollField.Add("Roll"); gyroRollFactor.Add(1f); break;
            }
        }
        if (MissileListener == null)
        {
            MissileListener = IGC.RegisterBroadcastListener(Radiotag);
        }

    // 只有成功抓到了核心组件（比如主控舱和陀螺仪），才宣布初始化成功
        if (MainCockpit != null && GyroList.Count > 0)
        {
            IsInitialized = true;
            Echo("== 导弹所有物理与无线电组件初始化成功 ==");
        }
    }
    //从母船更新SavedTargetPosition
    void GetTarget(){
        if (MissileListener != null && MissileListener.HasPendingMessage)
        {
        MyIGCMessage message = MissileListener.AcceptMessage();
        if (message.Tag == Radiotag)
        {
            Vector3D targetPosition;
            if (Vector3D.TryParse(message.Data.ToString(), out targetPosition))
            {
                if (targetPosition.LengthSquared() > 0)
                {
                    SavedTargetPosition = targetPosition;
                }
            }
        }
        }
    }

}

public class PIDController
{
    private double kP, kI, kD;
    private double maxIntegral;
    private double integralSum = 0;
    private double lastError = 0;
    private bool firstRun = true;

    public PIDController(double p, double i, double d, double maxI)
    {
        kP = p; kI = i; kD = d;
        maxIntegral = maxI;
    }

    public double Control(double error)
    {
        // P 项 (Proportional)：直接反映当前误差大小
        double pTerm = kP * error;

        // I 项 (Integral)：累加历史静差，带严格的抗饱和限制
        integralSum += error * (1.0 / 60.0); // 1格执行时间通常是1/60秒
        integralSum = MathHelper.Clamp(integralSum, -maxIntegral, maxIntegral); 
        double iTerm = kI * integralSum;

        // D 项 (Derivative)：预测未来趋势。计算误差变化率，充当防过冲的天然“物理阻尼器”
        double dTerm = 0;
        if (!firstRun)
        {
            double errorDerivative = WrapAngle((error - lastError)) * 60.0;
            dTerm = kD * errorDerivative;
        }
        else
        {
            firstRun = false;
        }

        lastError = error;

        // 总控制期望输出值
        return pTerm + iTerm + dTerm;
    }

    public void Reset()
    {
        integralSum = 0;
        lastError = 0;
        firstRun = true;
    }
    double WrapAngle(double a)
    {
        while (a > Math.PI)
            a -= Math.PI * 2.0;

        while (a <= -Math.PI)
            a += Math.PI * 2.0;

        return a;
    }
}


}