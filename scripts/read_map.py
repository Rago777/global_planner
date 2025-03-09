import numpy as np
import csv
import matplotlib.pyplot as plt

def read_costmap_from_csv(filename="costmap.csv"):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        # 跳过表头信息
        header = next(reader)
        width = int(header[1])
        height = int(header[3])

        data = []
        for row in reader:
            # 将每行转换为整数列表
            data.append([int(value) for value in row if value])  # 忽略空字符串
        
        # 确保数据是正确的二维数组形式
        costmap = np.array(data)
        if costmap.shape != (height, width):
            raise ValueError("The loaded costmap dimensions do not match the expected dimensions.")
        
        return costmap

def plot_costmap(costmap):
    plt.figure(figsize=(10, 10))
    # 使用imshow绘制成本图，cmap参数设置颜色映射
    # cmap='gray'将产生从黑到白的颜色渐变，你可以选择其他适合的颜色映射
    plt.imshow(costmap, cmap='gray', vmin=0, vmax=255)  # 假设OccupancyGrid的值范围是0-100
    plt.colorbar(label='Cost')
    plt.title('Costmap Visualization')
    plt.xlabel('X [cells]')
    plt.ylabel('Y [cells]')
    plt.show()

if __name__ == "__main__":
    try:
        costmap_data = read_costmap_from_csv("costmap.csv")
        plot_costmap(costmap_data)
    except Exception as e:
        print("Error: ", e)