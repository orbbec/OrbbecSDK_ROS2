import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np

def process_data(data):
    # 处理数据，将含有 "ms" 的数据转换为浮点数
    for col in ['min', 'avg', 'max']:
        data[col] = data[col].str.replace(' ms', '').astype(float)
    return data

def split_data(data):
    # 分别筛选出age和period的数据
    age_data = data[data['message_type'] == 'age']
    period_data = data[data['message_type'] == 'period']
    return age_data, period_data

def plot_data(data, title, y_ticks):
    plt.figure(figsize=(12, 6))
    
    for col, color in zip(['min', 'avg', 'max'], ['green', 'yellow', 'cyan']):
        # Convert datetime to numpy array
        x = data['_time'].to_numpy()
        y = data[col].to_numpy()
        plt.plot(x, y, label=col, color=color, marker='o', linestyle='-')
    
    plt.ylabel('Milliseconds')
    plt.title(title)
    
    plt.gca().xaxis.set_major_locator(mdates.MinuteLocator(interval=10))
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    
    plt.legend()
    plt.grid(True, which='both')
    plt.xticks(rotation=45)
    plt.yticks(range(0, y_ticks[1], y_ticks[0]))
    plt.tight_layout()
    plt.show()

def main():
    plt.style.use('dark_background')
    
    data = pd.read_csv('statistics.csv')
    
    data['_time'] = pd.to_datetime(data['_time'])
    
    data = process_data(data)
    age_data, period_data = split_data(data)

    plot_data(age_data, 'Message Age', (100, 501))

    plot_data(period_data, 'Message Period', (20, 101))

if __name__ == "__main__":
    main()
