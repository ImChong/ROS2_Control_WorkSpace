import numpy as np
import joblib
import pickle
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

my_dict = joblib.load("scripts/read_csv/openloong/taiji_middle1_modified.pkl")
dof_pos = my_dict['openloong_dance']['dof_pos']

num_frames = my_dict['openloong_dance']['dof_pos'].shape[0]
# my_dict['A2_dance']['pose_aa'] = my_dict['A2_dance']['pose_aa'].cpu().numpy()# my_dict['A2_taiji'] = {}
# for key, value in my_dict['A2_walk'].items():
#     if key != 'fps':
#         value = value[1:, ...]
#     my_dict['A2_taiji'][key] = value

# my_dict['A2_taiji'] = my_dict['A2_walk']
# my_dict.pop('A2_walk')
# my_dict['A2_dance']['contact_flags'] = {}
contact_flags = np.zeros((num_frames, 2)) 

contact_flags[0:290] = [1, 1]
contact_flags[290:550] = [0, 1]
contact_flags[550:3380] = [1, 1]
contact_flags[3380:3580] = [0, 1]
contact_flags[3580:4320] = [1, 1]
contact_flags[4320:4440] = [1, 0]
contact_flags[4440:4980] = [1, 1]
contact_flags[4980:5220] = [1, 0]
contact_flags[5220:6270] = [1, 1]
contact_flags[6270:6350] = [0, 1]
contact_flags[6350:6720] = [1, 1]
contact_flags[6720:7080] = [1, 0]

contact_flags[7080:9230] = [1, 1]
contact_flags[9230:9300] = [0, 1]
contact_flags[9300:10470] = [1, 1]
contact_flags[10470:10730] = [0, 1]
contact_flags[10730:11940] = [1, 1]
contact_flags[11940:12060] = [1, 0]
contact_flags[12060:12480] = [1, 1]
contact_flags[12480:12660] = [1, 0]
contact_flags[12660:13200] = [1, 1]
contact_flags[13200:13320] = [0, 1]
contact_flags[13320:num_frames] = [1, 1]


# my_dict['A2_walk']['root_trans_offset'][:,2] += 0.08
# my_dict['openloong_dance']['root_trans_offset'][:,2] += 0.02
my_dict['openloong_dance']['contact_flags'] = contact_flags
# my_dict['A2_dance']['root_trans_offset'] = my_dict['A2_dance']['root_trans_offset'][600:, ...]
# # my_dict['A2_dance']['pose_aa'] = my_dict['A2_dance']['pose_aa'][0, 12:1083, ...]
# my_dict['A2_dance']['dof_pos'] = my_dict['A2_dance']['dof_pos'][600:, ...]
# my_dict['A2_dance']['root_rot'] = my_dict['A2_dance']['root_rot'][600:, ...]
# my_dict['openloong_dance']['root_trans_offset'][:,2] += 0.02

# for key, value in my_dict['A2_dance'].items():
#     if key != 'fps' and key != 'pose_aa':
#         value = value[2:, ...]
#     my_dict['A2_dance'][key] = value

# my_dict['A2_dance']['pose_aa'] = my_dict['A2_dance']['pose_aa'][0, 2:, ...]
# my_dict['A2_dance']['pose_aa'] = my_dict['A2_dance']['pose_aa'][300:-300, ...]
# my_dict['openloong_dance']['dof_pos'] = my_dict['openloong_dance']['dof_pos'][80:, ...]
# my_dict['openloong_dance']['root_rot'] = my_dict['openloong_dance']['root_rot'][80:, ...]
# my_dict['openloong_dance']['root_trans_offset'] = my_dict['openloong_dance']['root_trans_offset'][80:, ...]
# my_dict['openloong_dance']['root_trans_offset'][:,0:2] = my_dict['openloong_dance']['root_trans_offset'][:,0:2]- my_dict['openloong_dance']['root_trans_offset'][0,0:2]

# my_dict['A2_dance']['contact_flags'] = my_dict['A2_dance']['contact_flags'][300:-300, ...]

# my_dict['A2_dance']['root_trans_offset'] = np.vstack((np.tile(my_dict['A2_dance']['root_trans_offset'][0], (180, 1)), 
#                                                       my_dict['A2_dance']['root_trans_offset'], np.tile(my_dict['A2_dance']['root_trans_offset'][-1], (180, 1))))
# my_dict['A2_dance']['dof_pos'] = np.vstack((np.tile(my_dict['A2_dance']['dof_pos'][0:1], (180, 1)), 
#                                                       my_dict['A2_dance']['dof_pos'], np.tile(my_dict['A2_dance']['dof_pos'][-2:-1], (180, 1))))
# my_dict['A2_dance']['root_rot'] = np.vstack((np.tile(my_dict['A2_dance']['root_rot'][0], (180, 1)), 
#                                                       my_dict['A2_dance']['root_rot'], np.tile(my_dict['A2_dance']['root_rot'][-1], (180, 1))))
# my_dict['A2_dance']['contact_flags'] = np.vstack((np.tile(my_dict['A2_dance']['contact_flags'][0], (180, 1)), my_dict['A2_dance']['contact_flags'], np.tile(my_dict['A2_dance']['contact_flags'][-1], (180, 1))))
# my_dict['A2_dance']['pose_aa'] = np.vstack((np.tile(my_dict['A2_dance']['pose_aa'][0], (300, 1,1)), 
#                                                       my_dict['A2_dance']['pose_aa'], np.tile(my_dict['A2_dance']['pose_aa'][-1], (300, 1,1))))

# for i in range(joint_pos.shape[0]):
#     print(*joint_pos[i][12:])  # Print each row in one line
# # print()  # Add a newline after printing all positions
# # ... existing code ...
# # my_dict['0-select_test_15_13_stageii']['fps'] = 30
with open('scripts/read_csv/openloong/taiji_middle1_modified.pkl', 'wb') as f:
    pickle.dump(my_dict, f)

print("Dictionary updated and saved to 'squat_with_transition_30fps.pkl'.")