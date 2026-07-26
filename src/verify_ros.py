import rospy
import roslib
roslib.load_manifest('ut_multirobot_sim')
roslib.load_manifest('amrl_msgs')
from ut_multirobot_sim.srv import utmrsStepper, utmrsStepperRequest

def main():
    rospy.init_node('verify_params_node')
    rospy.wait_for_service('utmrsStepper')
    step_service = rospy.ServiceProxy('utmrsStepper', utmrsStepper)
    
    print("Sending normal step (default parameters)...")
    req = utmrsStepperRequest()
    req.actions = [0]
    req.action_vel_x = [0.0]
    req.action_vel_y = [0.0]
    req.action_vel_angle = [0.0]
    req.max_speed = [-1.0]
    req.obstacle_margin = [-999.0]
    req.max_clearance = [-999.0]
    req.clearance_weight = [-999.0]
    req.carrot_dist = [-999.0]
    req.messages = ["test"]
    
    res = step_service(req)
    # Check velocity
    print(f"Normal Step -> Robot Vel X: {res.robot_responses[0].robot_vels[0].x:.4f}")
    
    print("\nSending extreme step (max_speed=0.1, obstacle_margin=5.0)...")
    req.max_speed = [0.1]
    req.obstacle_margin = [5.0]
    
    # Step a few times to let it accelerate/decelerate
    for _ in range(5):
        res = step_service(req)
        
    print(f"Extreme Step -> Robot Vel X: {res.robot_responses[0].robot_vels[0].x:.4f}")
    
if __name__ == '__main__':
    main()
