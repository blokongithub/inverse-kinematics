import numpy as np

def twodimension_inverse_kinematics(arm_lengths, coordinates):
    a, b = arm_lengths
    x, y = coordinates
    
    distance = np.sqrt(x**2 + y**2)
    
    if distance > (a + b) or distance < abs(a - b):
        raise ValueError("Coordinates out of reach")

    cos_theta = (a**2 + b**2 - distance**2) / (2 * a * b)
    theta = np.arccos(cos_theta)
    
    cos_beta = (a**2 + distance**2 - b**2) / (2 * a * distance)
    beta = np.arccos(cos_beta)
    
    A = np.arctan2(y, x) - beta
    
    B = np.pi - theta
    
    A_deg = np.degrees(A)
    B_deg = np.degrees(B)
    
    if A_deg < B_deg:
        A = np.arctan2(y, x) + beta
        A_deg = np.degrees(A)
        B_deg = 180 - B_deg

    return A_deg, B_deg #A_deg is angle from x axis to the line, B_deg is angle on the inside of the triangle

def threedimension_inverse_kinematics(lengths, pos):
    if len(pos) != 3 or len(lengths) != 2:
        raise AttributeError("Wrong list attributes")
    
    L1, L2 = lengths
    x, y, z = pos
    
    if z < 0:
        raise ValueError("Cannot have z less than 0")
    if x == 0 and y == 0:
        raise ValueError("x and y cannot be set to 0")
    
    xy_plane_hypotenuse = np.sqrt(x**2 + y**2)
    
    cos_rotation = (y**2 + xy_plane_hypotenuse**2 - x**2)/(2*y*xy_plane_hypotenuse)
    cos_rotation = np.clip(cos_rotation, -1, 1)
    rotation = np.degrees(np.arccos(cos_rotation))
    
    if x < 0:
        rotation = 360-rotation
        
        
    try: 
        A_deg, B_deg = twodimension_inverse_kinematics(lengths, [xy_plane_hypotenuse, z])
    except:
        raise ValueError("Coordinates are most likely out of reach")
    
    return round(rotation, 3), A_deg, B_deg


print(threedimension_inverse_kinematics([200, 150], [0.001, 0.001, 50]))
