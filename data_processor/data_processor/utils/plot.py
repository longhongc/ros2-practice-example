import matplotlib.pyplot as plt

fig = plt.figure(figsize=(8, 8), dpi=80)

repeat_length = 50

def temperature_plot_setting():
    global repeat_length
    fig.canvas.set_window_title('Temperature')
    repeat_length = 50
    plt.xlim(0, repeat_length)
    plt.ylim(0, 50)
    plt.xlabel('Index')
    plt.ylabel('Temperature °C')

prev_x = 0
prev_y = 0
def draw_temp(x, y):
    global repeat_length, prev_x, prev_y
    if(x > repeat_length):
        plt.xlim(x - repeat_length, x)
    else:
        plt.xlim(0, repeat_length)

    plt.plot([prev_x, x], [prev_y, y],  'ob-')
    plt.pause(0.0001)

    prev_x = x
    prev_y = y 
 
def speed_plot_setting():
    global repeat_length
    fig.canvas.set_window_title('Speed')
    ax1 = plt.subplot(2,1,1)
    ax2 = plt.subplot(2,1,2)
    ax1.title.set_text('linear')
    ax2.title.set_text('angular')
    repeat_length = 50
    ax1.set_xlim(0, repeat_length)
    ax2.set_xlim(0, repeat_length)
    ax1.set_ylim(-2, 6)
    ax2.set_ylim(-3.14, 3.14)
    return ax1, ax2

prev_x = 0
prev_y1 = 0
prev_y2 = 0
def draw_speed(x, y1, y2):
    global repeat_length, prev_x, prev_y1, prev_y2
    ax1 = plt.subplot(2,1,1)
    ax2 = plt.subplot(2,1,2)

    if(x > repeat_length):
        ax1.set_xlim(x - repeat_length, x)
        ax2.set_xlim(x - repeat_length, x)
    else:
        ax1.set_xlim(0, repeat_length)
        ax2.set_xlim(0, repeat_length)

    ax1.plot([prev_x, x], [prev_y1, y1],  '.b-')
    ax2.plot([prev_x, x], [prev_y2, y2], '.r-')
    plt.pause(0.0001)

    prev_x = x
    prev_y1 = y1 
    prev_y2 = y2 






