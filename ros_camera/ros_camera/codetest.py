import numpy as np
from scipy.spatial.transform import Rotation as R

array = np.array([20, 34, 56, 78, 1, 9], dtype=np.float32)

print(array.size) # Size of the array
print(array.itemsize) # Length of one array element in bytes,
print(array.nbytes) # Total bytes consumed by the elements of the array

arr = np.array([[0, 1, 0], [2, 0, 3], [0, 4, 0]]) # Example array
rows, cols = np.nonzero(arr) # Get row and column indices of non-zero elements
imax = np.argmax(rows)


RM = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
RM = R.from_matrix(RM)
RQ = RM.as_quat()
print(f"==>> RQ: \n{RQ}")