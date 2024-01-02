import yaml

goals_file = "config/waypoints1.yaml"
with open(goals_file, 'r') as file:
    goals = yaml.safe_load(file)
print(goals)
print(goals['goal1']['x'])
