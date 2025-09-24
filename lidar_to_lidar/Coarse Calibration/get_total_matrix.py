import numpy as np



# 变换矩阵格式 (4x4)
transform1 = np.array([
    [-0.75385628, 0.65640836, -0.02878847, -2.60161285],
    [0.65661462, 0.75422065, 0.00290681, 0.67902275],
    [0.02362091, -0.01671161, -0.99958130, 0.08904190],
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
