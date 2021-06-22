#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
import moveit_commander
from moveit_msgs.msg import *


class TestMoveAction(object):
    def __init__(self):
        super(TestMoveAction, self).__init__()

        # create a action client of move group
        self._move_client = SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)
        self._move_client.wait_for_server()

    def execute(self):
        # set joint name
        joint_names = ['Waist_Roll', 'Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']

        # prepare a joint trajectory
        goal = ExecuteTrajectoryGoal()
        goal.trajectory.joint_trajectory.joint_names = joint_names
        
        self.create_trajectory_points(goal)
        
        self._move_client.send_goal(goal)
    
    def create_trajectory_points(self, goal):
        '''
        saved_motion = [[0.0, 0.0, 0.0, -1.45, 0.0, 0.0, 0.0, 0.0],
        [1.331517426717457e-05, 5.589214870188798e-06, 3.9801453376033634e-06, -1.3722135761824332, -7.377725225847421e-06, -3.2476645797335854e-06, 1.7768139683175826e-05, -5.6452930503939506e-08],
        [2.663034853434914e-05, 1.1178429740377597e-05, 7.960290675206727e-06, -1.2944271523648665, -1.4755450451694842e-05, -6.495329159467171e-06, 3.553627936635165e-05, -1.1290586100787901e-07],
        [3.994552280152371e-05, 1.6767644610566392e-05, 1.1940436012810091e-05, -1.2166407285472998, -2.2133175677542263e-05, -9.742993739200756e-06, 5.330441904952748e-05, -1.6935879151181853e-07],
        [5.326069706869828e-05, 2.2356859480755193e-05, 1.5920581350413454e-05, -1.1388543047297333, -2.9510900903389684e-05, -1.2990658318934341e-05, 7.10725587327033e-05, -2.2581172201575802e-07],
        [6.657587133587286e-05, 2.794607435094399e-05, 1.990072668801682e-05, -1.0610678809121665, -3.688862612923711e-05, -1.623832289866793e-05, 8.884069841587915e-05, -2.8226465251969757e-07],
        [7.989104560304742e-05, 3.3535289221132785e-05, 2.3880872025620182e-05, -0.9832814570945998, -4.4266351355084525e-05, -1.9485987478401512e-05, 0.00010660883809905497, -3.3871758302363706e-07],
        [9.320621987022199e-05, 3.912450409132159e-05, 2.7861017363223547e-05, -0.9054950332770331, -5.164407658093195e-05, -2.27336520581351e-05, 0.0001243769777822308, -3.9517051352757655e-07],
        [0.00010652139413739655, 4.4713718961510386e-05, 3.184116270082691e-05, -0.8277086094594664, -5.902180180677937e-05, -2.5981316637868683e-05, 0.0001421451174654066, -4.5162344403151605e-07],
        [0.00011983656840457113, 5.0302933831699184e-05, 3.5821308038430275e-05, -0.7499221856418997, -6.63995270326268e-05, -2.9228981217602272e-05, 0.00015991325714858246, -5.080763745354556e-07],
         [0.00011966242163907737, 5.007498475606553e-05, 3.560578988981433e-05, -0.7499224543571472, -6.66150517645292e-05, -2.944451080111321e-05, 0.0001596977235749364, -7.275779694282392e-07],
        [0.00011091530174534355, 4.9751434956366814e-05, 3.882189755741921e-05, -0.8277044928792346, -6.150354611583881e-05, -1.7372543093127513e-05, 0.00014459900290498303, 6.747138048214968e-06],
        [0.00010216818185160971, 4.94278851566681e-05, 4.203800522502408e-05, -0.9054865314013221, -5.639204046714844e-05, -5.300575385141812e-06, 0.00012950028223502968, 1.4221854065858175e-05],
        [9.34210619578759e-05, 4.9104335356969384e-05, 4.525411289262896e-05, -0.9832685699234095, -5.1280534818458065e-05, 6.77139232284389e-06, 0.00011440156156507632, 2.169657008350138e-05],
        [8.467394206414206e-05, 4.878078555727067e-05, 4.847022056023383e-05, -1.061050608445497, -4.616902916976769e-05, 1.8843360030829587e-05, 9.930284089512296e-05, 2.917128610114459e-05],
        [7.592682217040823e-05, 4.845723575757195e-05, 5.1686328227838714e-05, -1.1388326469675845, -4.10575235210773e-05, 3.09153277388153e-05, 8.42041202251696e-05, 3.66460021187878e-05],
        [6.717970227667442e-05, 4.813368595787324e-05, 5.4902435895443584e-05, -1.2166146854896718, -3.594601787238693e-05, 4.298729544680099e-05, 6.910539955521625e-05, 4.4120718136431e-05],
        [5.843258238294059e-05, 4.781013615817452e-05, 5.8118543563048455e-05, -1.2943967240117593, -3.083451222369655e-05, 5.50592631547867e-05, 5.400667888526289e-05, 5.159543415407421e-05],
        [4.968546248920677e-05, 4.748658635847581e-05, 6.133465123065333e-05, -1.3721787625338466, -2.572300657500617e-05, 6.713123086277239e-05, 3.8907958215309534e-05, 5.9070150171717416e-05],
        [4.093834259547294e-05, 4.716303655877709e-05, 6.455075889825821e-05, -1.4499608010559342, -2.061150092631579e-05, 7.92031985707581e-05, 2.3809237545356163e-05, 6.654486618936063e-05]]               
        '''
        saved_motion = [[-1.5958316907926928e-06, 0.0, -0.0015339808305725455, -1.4557478427886963, 0.004601942375302315, -0.003067961661145091, -0.0015339808305725455, -0.0015339808305725455],
        [8.210593498855207e-06, 0.02222867795670819, -0.0008464670247450057, -1.4557445687874169, 0.004432173725191711, -0.0023827490266774675, -0.001532236332223854, -0.0013573255371405846],
        [1.8017018688503108e-05, 0.04445735591341638, -0.00015895321891746584, -1.4557412947861375, 0.004262405075081107, -0.0016975363922098444, -0.0015304918338751626, -0.0011806702437086238],
        [2.7823443878151006e-05, 0.06668603387012456, 0.0005285605869100739, -1.455738020784858, 0.0040926364249705026, -0.0010123237577422213, -0.0015287473355264714, -0.001004014950276663],
        [3.762986906779891e-05, 0.08891471182683276, 0.0012160743927376138, -1.4557347467835786, 0.0039228677748598985, -0.0003271111232745977, -0.00152700283717778, -0.0008273596568447021],
        [4.743629425744681e-05, 0.11114338978354095, 0.0019035881985651538, -1.455731472782299, 0.0037530991247492944, 0.00035810151119302584, -0.0015252583388290885, -0.0006507043634127411],
        [5.7242719447094705e-05, 0.13337206774024912, 0.0025911020043926933, -1.4557281987810196, 0.003583330474638691, 0.0010433141456606485, -0.001523513840480397, -0.0004740490699807803],
        [6.704914463674261e-05, 0.15560074569695734, 0.0032786158102202333, -1.4557249247797401, 0.0034135618245280863, 0.001728526780128273, -0.0015217693421317059, -0.00029739377654881935],
        [7.685556982639051e-05, 0.17782942365366552, 0.003966129616047773, -1.4557216507784607, 0.0032437931744174827, 0.0024137394145958956, -0.0015200248437830144, -0.00012073848311685859],
        [8.666199501603841e-05, 0.2000581016103737, 0.004653643421875313, -1.4557183767771813, 0.0030740245243068786, 0.0030989520490635187, -0.001518280345434323, 5.591681031510234e-05]]

        for i,positions in enumerate(saved_motion):
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.1 for j in range(len(positions))]
            pt.accelerations = [0.0 for j in range(len(positions))]
            pt.time_from_start = rospy.Duration(0.2 * i)
            goal.trajectory.joint_trajectory.points.append(pt)


if __name__ == '__main__': 
    rospy.init_node('test')
    moveit_action = TestMoveAction() 
    moveit_action.execute()