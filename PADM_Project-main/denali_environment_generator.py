# Generate environment
start = (-4,-2)
radius = 0.1
goal_regions = (Polygon([(12,3), (12,4), (13,4),(13,3)]), Polygon([(-4.5,4), (-3.5,4), (-4.5,3.5),(-3.5,3.5)]), Polygon([(9.5,0), (9.5,0.5), (10,0),(10,0.5)]), Polygon([(14,-3.5), (14,-3), (14.5,-3.5),(14.5,-3)]))
bounds = (-5, -4, 15, 5)
random.seed(101)
env = random_environment(bounds, start, radius, goal_regions, 600, (0.45, 0.45))
plot_environment(env)
env.save_to_yaml('Denali_600.yaml')