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

def frame(x, y):
    global repeat_length
    if(x > repeat_length):
        plt.xlim(x - repeat_length, x)
    else:
        plt.xlim(0, repeat_length)

    plt.scatter(x, y)
    plt.pause(0.0001)
