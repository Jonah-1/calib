import numpy as np

# 从transform_matrices.txt文件中读取变换矩阵
def read_transform_matrices(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # 解析source transform矩阵
    source_matrix_lines = []
    i = 1  # 跳过"source transform:"行
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

transform_source2=np.array([[0.99970985, 0.01768318, -0.01635512, 0.01567256],
  [-0.01797498, 0.99967876, -0.01787042, 0.06723239],
  [0.01603386, 0.01815922, 0.99970651, -0.06077816],
  [0.00000000, 0.00000000, 0.00000000, 1.00000000]])





transform_source=transform_source2@transform_source1



# ])

source_matrix_str = np.array2string(transform_source, separator=',')
print("source transform:\n{}\n".format(source_matrix_str))
np.save("transform.npy", transform_source)

# 格式化打印矩阵，添加逗号
matrix_str = np.array2string(transform_source, separator=', ')# 格式化打印矩阵，添加逗号
# 保存矩阵到txt文件
with open("transform_matrices.txt", "w") as f:
    f.write("transform:\n{}".format(matrix_str))
