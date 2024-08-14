import yaml

class HolaBotConfigReader:
    def __init__(self, file_path):
        self.file_path = file_path
        self.deadbands = {}

    def read_deadbands(self):
        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file)

        hola_bots = data.get('hola_bots', [])

        for bot in hola_bots:
            bot_id = bot.get('hola_bot_ID')
            dead_bands = bot.get('dead_bands', {})
            
            left_wheel = dead_bands.get('left_wheel', {})
            right_wheel = dead_bands.get('right_wheel', {})
            rear_wheel = dead_bands.get('rear_wheel', {})
            
            left_positive_rotation = left_wheel.get('positive_rotation', 0)
            left_negative_rotation = left_wheel.get('negative_rotation', 0)
            
            right_positive_rotation = right_wheel.get('positive_rotation', 0)
            right_negative_rotation = right_wheel.get('negative_rotation', 0)
            
            rear_positive_rotation = rear_wheel.get('positive_rotation', 0)
            rear_negative_rotation = rear_wheel.get('negative_rotation', 0)

            self.deadbands[bot_id] = {
                'left': {
                    '+': left_positive_rotation,
                    '-': left_negative_rotation
                },
                'right': {
                    '+': right_positive_rotation,
                    '-': right_negative_rotation
                },
                'rear': {
                    '+': rear_positive_rotation,
                    '-': rear_negative_rotation
                }
            }

    def get_deadbands(self):
        return self.deadbands

if __name__ == "__main__":
    file_path = "config.yaml"  # Update with your actual YAML file path
    reader = HolaBotConfigReader(file_path)
    reader.read_deadbands()

    deadbands = reader.get_deadbands()

    for bot_id, wheel_bands in deadbands.items():
        print(f"Hola Bot ID {bot_id}:")
        print(f"Left Wheel: Positive Rotation Dead Band = {wheel_bands['left']['+']}, Negative Rotation Dead Band = {wheel_bands['left']['-']}")
        print(f"Right Wheel: Positive Rotation Dead Band = {wheel_bands['right']['+']}, Negative Rotation Dead Band = {wheel_bands['right']['-']}")
        print(f"Rear Wheel: Positive Rotation Dead Band = {wheel_bands['rear']['+']}, Negative Rotation Dead Band = {wheel_bands['rear']['-']}")
        print("-" * 30)
