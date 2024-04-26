import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from data_operations import load_hdf, load_init_taxels_hdf


def main():
    data_set = '28021507'
    n_runs = 1

    numframes = 100
    init_taxel_positions = load_init_taxels_hdf(data_set)[0]
    n_pins = len(init_taxel_positions)

    color_data = np.random.random((numframes, n_pins))
    x, y, c = np.random.random((3, n_pins))

    fig = plt.figure()
    scat = plt.scatter(x, y, c=c, s=100)

    ani = animation.FuncAnimation(fig, update_plot, frames=range(numframes),
                                  fargs=(color_data, scat))
    plt.show()

def update_plot(i, data, scat):
    scat.set_array(data[i])
    return scat,

main()