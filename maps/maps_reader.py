import yaml

class ControllerJointsLoader:
    """
    This class loads the controller joints data from the simple_moveit_controllers.yaml file
    """
    def __init__(self, filename):
        self.filename = filename
        self.controller_data = {}
        self.load_controller_data()

    def load_controller_data(self):
        with open(self.filename, 'r') as file:
            data = yaml.safe_load(file)
            
        arm_count = 1
        hand_count = 1

        for controller in data['controller_list']:
            # Remove "_controller" from the name
            name = controller['name'].replace('_controller', '')
            # Extract the base name for eef_link
            eef_name = name.split('_')[0] + '_' + name.split('_')[1]
            # Create robot_arm or robot_hand key
            if 'arm' in name:
                planning_group = 'robot_arm_' + str(arm_count)
                arm_count += 1
            elif 'hand' in name:
                robot_number = name.split('_')[1]
                planning_group = 'robot_hand_' + robot_number
                hand_count += 1
            else:
                continue
            # Store the planning_group, its corresponding joints, controller name and eef_link in the dictionary
            self.controller_data[planning_group] = {
                'planning_group': name,
                'joints': controller['joints'],
                'eef_link': eef_name + '_link_6'
            }

    def get_controller_data(self):
        return self.controller_data

class RecipeLoader:
    def __init__(self, actions_file):
        self.actions_file = actions_file

    def read_instructions_from_yaml(self):
        with open(self.actions_file, 'r') as file:
            instructions_list = yaml.safe_load(file)

        # for instruction in instructions_list:
        #     instruction_name = instruction['name']
        #     parameters = instruction['parameters']
        
        return instructions_list

# Using the class
# filename = '/home/robotlab/Documents/Github/P10-MAP/src/kuka_config_multiple/config/simple_moveit_controllers.yaml' # Path to the yaml file
# controller_data = loader.get_controller_data()
# print(controller_data)