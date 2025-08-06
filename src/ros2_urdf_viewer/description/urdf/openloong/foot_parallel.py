from urdfpy import URDF
import numpy as np
from scipy.spatial.transform import Rotation as R
import joblib
from scipy.optimize import minimize
from tqdm import tqdm  # 在文件顶部添加导入

def get_child_link_by_joint(robot, joint_name):
    """根据关节名称获取其子链接对象"""
    # 遍历所有关节寻找目标
    for joint in robot.joints:
        if joint.name == joint_name:
            child_link_name = joint.child  # 关节的 child 属性即子链接名称
            return next(link for link in robot.links if link.name == child_link_name)
    raise ValueError(f"未找到名为 '{joint_name}' 的关节")

def compute_foot_roll(theta, base_tf, fixed_joint_angles, robot, joint_name):
    """
    计算给定 leg_l6_joint 角度时的足部 Roll 角
    """
    # 获取目标链接（通过关节名称）
    # target_joint = next(j for j in robot.joints if j.name == joint_name)
    target_link = get_child_link_by_joint(robot, joint_name)
    
    # 更新关节配置（仅改变 leg_l6_joint）
    joint_config = fixed_joint_angles.copy()
    joint_config[joint_name] = theta
    
    # 计算局部坐标系下的足部变换
    # target_link = next(link for link in robot.links if link.name == "leg_l6_link")
    local_tf = robot.link_fk(joint_config)[target_link]
    
    # 转换到全局坐标系
    global_tf = base_tf @ local_tf
    
    # 提取 Roll 角（使用 XYZ 欧拉角，第一个分量为 Roll）
    roll = R.from_matrix(global_tf[:3, :3]).as_euler('xyz')[0]
    return abs(roll)  # 优化目标为最小化 |Roll|

def optimize_foot_roll(urdf_file, dof_pos, root_rot, root_trans, optimized_joint):
    # 加载 URDF 模型
    robot = URDF.load(urdf_file)
    n_frames = dof_pos.shape[0]
    optimized_angles = np.copy(dof_pos)
    
    # 确定 leg_l6_joint 在数组中的索引
    joint_names = [
        'waist_joint_pitch', 'waist_joint_roll', 'waist_joint_yaw',
        'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint',
        'leg_l5_joint', 'leg_l6_joint',  # 索引8为 leg_l6_joint
        'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint',
        'leg_r5_joint', 'leg_r6_joint',  # 索引8为 leg_l6_joint
        'x_left_arm_joint1',
        'x_left_arm_joint2',
        'x_left_arm_joint3',
        'x_left_arm_joint4',
        'x_left_arm_joint5',
        'x_left_arm_joint6',
        'x_left_arm_joint7',
        'x_right_arm_joint1',
        'x_right_arm_joint2',
        'x_right_arm_joint3',
        'x_right_arm_joint4',
        'x_right_arm_joint5',
        'x_right_arm_joint6',
        'x_right_arm_joint7'
        # ... 其他关节
    ]
    optimized_joint_idx = joint_names.index(optimized_joint)
    
    progress_bar = tqdm(total=n_frames, desc=f"优化 {optimized_joint} 关节", unit="frame")

    for i in range(n_frames):
        # --------------------------
        # 步骤1：准备当前帧数据
        # --------------------------
        # 基座变换
        base_rot = R.from_quat(root_rot[i]).as_matrix()
        base_tf = np.eye(4)
        base_tf[:3, :3] = base_rot
        base_tf[:3, 3] = root_trans[i]
        
        # 固定其他关节的角度
        fixed_joint_angles = {
            name: dof_pos[i][joint_names.index(name)]
            for name in joint_names if name != optimized_joint
        }
        
        # --------------------------
        # 步骤2：定义优化问题
        # --------------------------
        # 初始猜测（当前关节角度）
        theta_init = dof_pos[i, optimized_joint_idx]
        
        # 目标函数（需传入额外参数）
        def objective(theta):
            return compute_foot_roll(theta, base_tf, fixed_joint_angles, robot, optimized_joint)
        
        # 执行优化
        result = minimize(
            objective,
            x0=theta_init,
            bounds=[(-0.44, 0.44)],  # 关节限位
            method='SLSQP',        # 带约束的优化算法
            tol=1e-6              # 收敛精度
        )
        
        # --------------------------
        # 步骤3：保存优化结果
        # --------------------------
        if result.success:
            optimized_angles[i, optimized_joint_idx] = result.x[0]
        else:
            print(f"警告：第 {i} 帧优化失败！")
            # 可回退到原始角度或插值
        progress_bar.update(1)
        progress_bar.set_postfix_str(f"当前角度: {result.x[0]:.3f} rad" if result.success else "优化失败")

    progress_bar.close()

    return optimized_angles

# 使用示例
if __name__ == "__main__":
    # 加载数据和模型
    urdf_path = 'resources/robots/openloong/urdf/openloong_no_head.urdf'
    data = joblib.load('data/imitation_data/openloong/openloong_dance_quan_with_aa.pkl')
    
    # 提取所需数据
    key = next(iter(data.keys()))
    dof_pos = data[key]['dof_pos']          # 关节角度 (n_frames, n_joints)
    root_rot = data[key]['root_rot']        # 基座旋转（四元数）(n_frames, 4)
    root_trans = data[key]['root_trans_offset']  # 基座平移 (n_frames, 3)
    
    # 计算足部朝向
    optimized_dof_pos = optimize_foot_roll(
        urdf_file=urdf_path,
        dof_pos=dof_pos,
        root_rot=root_rot,
        root_trans=root_trans,
        optimized_joint='leg_l6_joint'
    )
    
    optimized_dof_pos = optimize_foot_roll(
        urdf_file=urdf_path,
        dof_pos=optimized_dof_pos,
        root_rot=root_rot,
        root_trans=root_trans,
        optimized_joint='leg_r6_joint'
    )
    
    
    
    # 保存结果
    data[key]['dof_pos'] = optimized_dof_pos
    joblib.dump(data, 'data/imitation_data/openloong/openloong_dance_quan_with_aa_feet_parallel.pkl')