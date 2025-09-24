import numpy as np



# 变换矩阵格式 (4x4)
transform1 = np.array([
    [-0.75385628, 0.65640836, -0.02878847, -2.60161285],
    [0.65661462, 0.75422065, 0.00290681, 0.67902275],
    [0.02362091, -0.01671161, -0.99958130, 0.08904190],
    [0.00000000, 0.00000000, 0.00000000, 1.00000000]
])



transform=transform1

# 格式化打印矩阵
matrix_str = np.array2string(
    transform,
    separator=', ',
    formatter={'float_kind': lambda x: f"{x:.8f}"},  # 保留8位小数
    max_line_width=np.inf  # 确保矩阵在一行内打印
)

# 打印矩阵
print(matrix_str)

# 保存矩阵到文本文件
with open("transform_matrices.txt", "w") as f:
    f.write(matrix_str)

