import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
from csv import DictWriter
import pandas as pd


def animate(i):
    cvs_file_name = 'D:/Yedek Belgeler/WebotsProjects/repulsiveforce/controllers/main/velocity_values.csv'
    data = pd.read_csv(cvs_file_name)
    
    x = data['Time']
    y1 = data['Velocity']
    y2 = data['Angular_Velocity']
    
    plt.cla()
    plt.plot(y1, color='#D50909', label= 'Velocity')
    plt.plot(y2, color='#335EFF', label= 'Angular_Velocity')
    
    plt.legend(loc='upper left')
    plt.tight_layout()
    plt.xlabel("Time")
    #plt.ylabel("Velocity") 
    
ani = FuncAnimation(plt.gcf(), animate, interval = 1000)

plt.tight_layout()
plt.show()    
 