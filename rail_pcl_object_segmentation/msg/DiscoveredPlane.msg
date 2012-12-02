# Represents a plane that has been discovered in a point cloud

# The points that lie within the discovered plane
sensor_msgs/PointCloud2 planeCloud

# The model for the extracted plane
# In the form of plane equation coordinates
# e.g. ax + by + cz + d = 0
float64 a
float64 b
float64 c
float64 d
