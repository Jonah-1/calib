import numpy as np



# 变换矩阵格式 (4x4)
transform1 = np.array([
<<<<<<< HEAD
    [0.99926783, -0.03803681, 0.00412403, -0.04089091],
    [0.03800163, 0.99924322, 0.00829867, -0.43839133],
    [-0.00443657, -0.00813588, 0.99995706, 0.01545669],
=======
    [0.99952729, -0.00809751, -0.02965840, 0.07305451],
    [0.00802906, 0.99996482, -0.00242632, 0.02646650],
    [0.02967700, 0.00218704, 0.99955715, -0.14360571],
>>>>>>> 38a7e901473e2eb7cd9f5f07a3e750ff014220b6
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
