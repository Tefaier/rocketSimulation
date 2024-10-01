from Cameras import PrimitiveCamera


def get_system_rect(simulation_data, camera : PrimitiveCamera) -> ((float, float, float), (float, float, float)):  # (xmin, ymin, zmin), (xmax, ymax, zmax)
    max_x = min_x = simulation_data[0][1][0][1][0]
    max_y = min_y = simulation_data[0][1][0][1][1]
    max_z = min_z = simulation_data[0][1][0][1][2]
    for time, data in simulation_data:
        for object in data:
            name = object[0]
            coords = object[1]
            max_x = max(max_x, coords[0])
            min_x = min(min_x, coords[0])
            max_y = max(max_y, coords[1])
            min_y = min(min_y, coords[1])
            max_z = max(max_z, coords[2])
            min_z = min(min_z, coords[2])
            #print(name, coords[0], coords[1])
    return (min_x, min_y, min_z), (max_x, max_y, max_z)