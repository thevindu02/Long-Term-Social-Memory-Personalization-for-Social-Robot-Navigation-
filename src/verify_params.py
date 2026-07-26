import os
import sys
import numpy as np

# Setup path
sys.path.append(os.getcwd())

from src.environment.scenarios.common_scenarios import envs_hospital_ward
from src.environment.ros_social_gym import RosSocialEnv

from src.environment.observers import DefaultObserver
from src.environment.rewarders import DefaultRewarder

def main():
    scenario, _ = envs_hospital_ward()
    scenario.generate_scenario(num_humans=0, num_agents=1)
    
    observer = DefaultObserver()
    rewarder = DefaultRewarder()
    
    env = RosSocialEnv(
        observer=observer, rewarder=rewarder, scenarios=[scenario], 
        num_humans=0, num_agents=1, debug=True
    )
    
    # We use debug=True so action=0 (GoAlone) is forced always.
    
    obs = env.reset()
    print("Initial reset done.")
    
    # Step 10 times normally (sentinels)
    print("--- NORMAL STEPS (Sentinels -999.0) ---")
    for _ in range(10):
        # We pass dummy action, debug=True overrides it
        obs, rewards, dones, infos = env.step({env.agents[0]: 0})
        # the info dictionary contains 'player_0' with its velocity and shortest_path
        p_info = infos['player_0']
        print(f"Vel: {p_info['velocity']:.4f}, Pos: {p_info['position']}")
        
    print("\n--- INJECTING EXTREME PARAMETERS (-999.0 overridden manually in script) ---")
    # We will hack the env to send a tiny max_speed and huge obstacle_margin
    # We just do it for 10 steps
    # We have to patch sim_step temporarily to inject different max_speed and margin
    original_sim_step = env.sim_step
    def hooked_sim_step(args):
        # args[4] = max_speeds
        # args[5] = obstacle_margin
        # args[6] = max_clearance
        # args[7] = clearance_weight
        # args[8] = carrot_dist
        args[4] = [0.1] * env.curr_num_agents
        args[5] = [5.0] * env.curr_num_agents
        return original_sim_step(args)
        
    env.sim_step = hooked_sim_step
    
    for _ in range(10):
        obs, rewards, dones, infos = env.step({env.agents[0]: 0})
        p_info = infos.get('player_0', infos.get('robot_data', {}))
        print(f"Vel: {p_info['velocity']:.4f}, Pos: {p_info['position']}")

if __name__ == '__main__':
    main()
