import numpy as np

# 从transform_matrices.txt文件中读取变换矩阵
def read_transform_matrices(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # 解析transform矩阵
    source_matrix_lines = []
    i = 0  # 
    while i < len(lines):
        if lines[i].strip():  # 忽略空行
            source_matrix_lines.append(lines[i].strip())
        i += 1
    
    
    # 将字符串转换为numpy数组
    source_matrix_str = ''.join(source_matrix_lines).replace('[', '').replace(']', '')
    source_values = np.fromstring(source_matrix_str, sep=',')
    source_transform = source_values.reshape(4, 4)
    
    
    return source_transform

# 从文件读取变换矩阵
transform_source1 = read_transform_matrices('transform_matrices.txt')

transform_source2=np.array([[0.99960345, 0.01759227, -0.02199011, 0.06911865],
  [-0.01770544, 0.99983095, -0.00496248, 0.00524328],
  [0.02189910, 0.00534985, 0.99974597, -0.02494814],
  [0.00000000, 0.00000000, 0.00000000, 1.00000000]])





transform_source=transform_source2@transform_source1



# ])

# 格式化打印矩阵
matrix_str = np.array2string(
    transform_source,
    separator=', ',
    formatter={'float_kind': lambda x: f"{x:.8f}"},  # 保留8位小数
    max_line_width=np.inf  # 确保矩阵在一行内打印
)

# 打印矩阵
print(matrix_str)

# 保存矩阵到文本文件
with open("transform_matrices.txt", "w") as f:
    f.write(matrix_str)
