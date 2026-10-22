import matplotlib.pyplot as plt

class Plot:
    def __init__(self,borderleft,borderright):
        plt.plot(borderleft[:, 0], borderleft[:, 1], color="GRAY")
        plt.plot(borderright[:, 0], borderright[:, 1], color="GRAY")

    def plot_cartrace(self, cartrace):
        plt.plot(cartrace[:,0],cartrace[:,1], color="BLUE")
        

    def plot_speed(self, cartrace, velocity_list):
        sc = plt.scatter(
            cartrace[:, 0],
            cartrace[:, 1],
            c=velocity_list,
            cmap="coolwarm",  
            s=3
        )
        cbar = plt.colorbar(sc)
        cbar.set_label("speed [m/s]")
        

    def plot_midline(self,path):
        plt.plot(path[:, 0], path[:, 1], color="RED")


    def show(self):
        plt.show()