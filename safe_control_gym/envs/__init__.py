"""Register environments.

"""
from safe_control_gym.utils.registration import register

register(id="cartpole",
         entry_point="safe_control_gym.envs.gym_control.cartpole:CartPole",
         config_entry_point="safe_control_gym.envs.gym_control:cartpole.yaml")

register(id="quadrotor",
         entry_point="safe_control_gym.envs.gym_pybullet_drones.quadrotor:Quadrotor",
         config_entry_point="safe_control_gym.envs.gym_pybullet_drones:quadrotor.yaml")

register(id="hexarotor",
         entry_point="safe_control_gym.envs.gym_pybullet_drones.hexarotor:Hexarotor2D",
         config_entry_point="safe_control_gym.envs.gym_pybullet_drones:hexarotor.yaml")

register(id="hexarotor_tracking",
         entry_point="safe_control_gym.envs.gym_pybullet_drones.hexarotor:Hexarotor2D",
         config_entry_point="safe_control_gym.envs.gym_pybullet_drones:hexarotor_tracking.yaml")

register(id="hexarotorbig_tracking",
         entry_point="safe_control_gym.envs.gym_pybullet_drones.hexarotorbig:Hexarotor2DBig",
         config_entry_point="safe_control_gym.envs.gym_pybullet_drones:hexarotorbig_tracking.yaml")