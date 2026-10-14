# This represents a point in free space with uncertainty.

Point point

# Row-major representation of the 3x3 covariance matrix
# In order, the parameters are:
# (x, y, z)
float64[9] covariance
