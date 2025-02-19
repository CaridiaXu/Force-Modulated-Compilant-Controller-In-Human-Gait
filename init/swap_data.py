def swap_columns_in_file(input_file, output_file):
    # 读取文件内容
    with open(input_file, 'r') as file:
        lines = file.readlines()
    
    # 处理每一行的数据，交换第一列和第二列
    swapped_lines = []
    for line in lines:
        columns = line.split()
        if len(columns) > 1:
            # 互换第一列和第二列
            columns[1], columns[2] = columns[2], columns[1]
        swapped_lines.append('\t'.join(columns))
    
    # 写回文件或输出到新的文件
    with open(output_file, 'w') as file:
        for swapped_line in swapped_lines:
            file.write(swapped_line + '\n')

# 假设输入文件是 "test.txt" 输出到 "swapped_test.txt"
swap_columns_in_file('0049_90.258_1.152.par', 'PD_perturbation_test.par') # PD
#swap_columns_in_file('FMC_perturbation_test.par', 'FMC_perturbation_test.par') # FMC
#swap_columns_in_file('code/init/1872_52.186_1.348.par', 'code/init/H1922GaitRS2Hfd4_7_best.par') 

