import yaml
import numpy as np
with open('camera_lidar_calibration_1.yaml', 'r') as f:
    data = yaml.load(f)

print(type(data))
print(data)

# write dictionary to yaml
# import yaml
# data = {'name':'johnson', 'age':23,
#         'spouse':{'name':'Hallie', 'age':23},
#         'children':[{'name':'Jack', 'age':2}, {'name':'Linda', 'age':2}]}
#
# with open('test2.yaml','w') as f:
#     f.write(yaml.dump(data))
#     print(yaml.dump(data))
