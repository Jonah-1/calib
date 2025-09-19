import numpy as np



# 变换矩阵格式 (4x4)
transform1 = np.array([
    [0.99926783, -0.03803681, 0.00412403, -0.04089091],
    [0.03800163, 0.99924322, 0.00829867, -0.43839133],
    [-0.00443657, -0.00813588, 0.99995706, 0.01545669],
    [0.00000000, 0.00000000, 0.00000000, 1.00000000]
])



transform=transform1


# 格式化打印矩阵，添加逗号
matrix_str = np.array2string(transform, separator=', ')


print("transform:\n{}".format(matrix_str))

# 保存矩阵到npy文件
np.save("transform.npy", transform)

# 保存矩阵到txt文件
with open("transform_matrices.txt", "w") as f:
    f.write("transform:\n{}".format(matrix_str))
