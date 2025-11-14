# trajectory_point.msg

builtin_interfaces/Time time_from_start

# Posición (x, y, z)
geometry_msgs/Point position

# Orientación (roll, pitch, yaw)
geometry_msgs/Vector3 orientation # O puedes usar un Quaternion

# Velocidad Lineal (vx, vy, vz)
geometry_msgs/Vector3 velocity_linear

# Velocidad Angular (v_roll, v_pitch, v_yaw)
geometry_msgs/Vector3 velocity_angular

# Aceleración Lineal
geometry_msgs/Vector3 acceleration_linear

# Aceleración Angular
geometry_msgs/Vector3 acceleration_angular