import pandas as pd
import numpy as np

def read_param_file(filename):
    """
    读取参数文件并返回DataFrame
    文件格式：参数名 best_value mean_value std_dev
    """
    # 读取文件，跳过可能的空行
    df = pd.read_csv(filename, delimiter='\s+', header=None, skip_blank_lines=True)
    
    # 设置列名
    df.columns = ['param_name', 'best_value', 'mean_value', 'std_dev']
    
    return df

def set_fixed_std(df, new_std_value):
    """
    将所有标准差设置为指定的固定值
    """
    df_modified = df.copy()
    df_modified['std_dev'] = new_std_value
    return df_modified

def set_std_as_fraction_of_mean(df, fraction=0.1):
    """
    将标准差设置为均值的某个比例
    """
    df_modified = df.copy()
    df_modified['std_dev'] = df_modified['mean_value'].abs() * fraction
    return df_modified

def write_param_file(df, output_filename):
    """
    将修改后的数据写回文件，保持原始格式
    """
    # 使用固定格式写入文件
    with open(output_filename, 'w') as f:
        for _, row in df.iterrows():
            # 对于较大的数值使用普通格式，对于很小的数值使用科学计数法
            if abs(row['std_dev']) >= 0.01:
                std_str = f"{row['std_dev']:.8f}"
            else:
                std_str = f"{row['std_dev']:.8e}"
                
            # 确保参数名对齐（假设最长参数名为50个字符）
            param_name_padded = f"{row['param_name']:<50}"
            
            # 写入行，保持原始格式
            f.write(f"{param_name_padded}\t{row['best_value']:.8f}\t{row['mean_value']:.8f}\t{std_str}\n")

def main():
    # 示例用法
    input_file = "FMC_perturbation_test.par"
    
    # 读取参数文件
    df = read_param_file(input_file)
    
    # 方法1：设置固定的标准差值
    fixed_std = 0.001
    df_fixed = set_fixed_std(df, fixed_std)
    write_param_file(df_fixed, "FMC_perturbation_test_fixed_std.par")
    
    # 方法2：将标准差设置为均值的0.002
    df_fraction = set_std_as_fraction_of_mean(df, fraction=0.005)
    write_param_file(df_fraction, "FMC_perturbation_test_fraction_std.par")

if __name__ == "__main__":
    main()