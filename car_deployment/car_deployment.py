import torch
import numpy as np
import glob

def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * \
        torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
            shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

def yaw_quat(quat: torch.Tensor) -> torch.Tensor:
    quat_yaw = quat.clone().view(-1, 4)
    qx = quat_yaw[:, 0]
    qy = quat_yaw[:, 1]
    qz = quat_yaw[:, 2]
    qw = quat_yaw[:, 3]
    yaw = torch.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    quat_yaw[:, :2] = 0.0
    quat_yaw[:, 2] = torch.sin(yaw / 2)
    quat_yaw[:, 3] = torch.cos(yaw / 2)
    quat_yaw = normalize(quat_yaw)
    return quat_yaw

def normalize(x, eps: float = 1e-9):
    return x / x.norm(p=2, dim=-1).clamp(min=eps, max=None).unsqueeze(-1)

def quat_apply(a, b):
    shape = b.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 3)
    xyz = a[:, :3]
    t = xyz.cross(b, dim=-1) * 2
    return (b + a[:, 3:] * t + xyz.cross(t, dim=-1)).view(shape)

def wrap_to_pi(angles):
    angles %= 2*np.pi
    angles -= 2*np.pi * (angles > np.pi)
    return angles

class CarDeploy:
    def __init__(self,):

        self.projected_gravity=torch.zeros(1, 3, dtype=torch.float)
        self.commands = torch.zeros(1, 3, dtype=torch.float)
        self.gravity_vec = torch.Tensor([[0., 0., -1.]])
        self.forward_vec = torch.Tensor([[1., 0., 0.]])

        self.max_episode_length_s=9
        self.obs_scales_dof_vel=0.2

        self.dt=0.2
        self.timer_left=torch.Tensor([9])

    def get_data(self,
                 base_ang_vel,      # 车身三轴角速度
                 base_quat,         # 车身姿态四元数
                 car_pos,           # 车身位置
                 position_targets,  # 目标位置
                 _target_heading,   # 目标朝向
                 dof_pos,           # 四电机位置
                 dof_vel,           # 四电机速度
                 actions,           # 模型算出的四电机速度
                 ray2d_,):          # 激光雷达数据
        self.base_ang_vel = base_ang_vel
        self.base_quat = base_quat
        self.car_pos = car_pos
        self.position_targets = position_targets
        self._target_heading = _target_heading
        self.dof_pos = dof_pos
        self.dof_vel = dof_vel
        self.actions = actions
        self.ray2d_ = ray2d_

    def get_gravity(self):
        self.projected_gravity = quat_rotate_inverse(self.base_quat, self.gravity_vec)

    def get_commands(self):
        pos_diff = self.position_targets - self.car_pos
        self.commands[:, :2] = quat_rotate_inverse(yaw_quat(self.base_quat[:]), pos_diff)[:, :2]  # only x, y used here

        forward = quat_apply(self.base_quat, self.forward_vec)
        heading = torch.atan2(forward[:, 1], forward[:, 0])
        self.heading_targets = wrap_to_pi(
            self._target_heading + torch.atan2(pos_diff[:, 1:2], pos_diff[:, 0:1]))
        self.commands[:, 2] = wrap_to_pi(self.heading_targets[:, 0] - heading)

    def compute_observations(self):
        """ Computes observations
        """
        self.get_gravity()
        self.get_commands()
        self.timer_left-=self.dt

        self.obs_buf = torch.cat((
            self.base_ang_vel,  # 0:3
            self.projected_gravity,  # 3:6
            self.commands[:, :3],  # 6:9
            self.timer_left.unsqueeze(1) / self.max_episode_length_s,  # 9:10
            self.dof_pos,  # 10:14
            self.dof_vel * self.obs_scales_dof_vel,  # 14:18
            self.actions  # 18:22
        ), dim=-1)  # append ray2d obs after this, 50:
        # print(self.timer_left.unsqueeze(1) / self.max_episode_length_s)
        # add perceptive inputs if not blind

        self.obs_buf = torch.cat((self.obs_buf, self.ray2d_), dim=-1)

if __name__ == '__main__':
    dirs = glob.glob(f"policy/*")
    policy = torch.jit.load(dirs[0])
    car=CarDeploy()

    while(True):
        base_ang_vel=torch.zeros([1,3])  # 车身三轴角速度
        base_quat=torch.zeros([1,4])  # 车身姿态四元数
        car_pos=torch.zeros([1,3])  # 车身位置
        position_targets=torch.zeros([1,3])  # 目标位置
        _target_heading=torch.zeros([1])  # 目标朝向
        dof_pos=torch.zeros([1,4])  # 四电机位置
        dof_vel=torch.zeros([1,4])  # 四电机速度
        actions=torch.zeros([1,4])  # 模型算出的四电机速度
        ray2d_=torch.zeros([1,11])  # 激光雷达数据

        car.get_data(
                 base_ang_vel,      # 车身三轴角速度
                 base_quat,         # 车身姿态四元数
                 car_pos,           # 车身位置
                 position_targets,  # 目标位置
                 _target_heading,   # 目标朝向
                 dof_pos,           # 四电机位置
                 dof_vel,           # 四电机速度
                 actions,           # 模型算出的四电机速度
                 ray2d_)
        car.compute_observations()

        actions=policy(car.obs_buf)
        print(actions)
