from matplotlib import pyplot as plt


datasets = ["A", "B", "C", "D"]


for dataset in datasets:
    try:
        with open(f"loading/{dataset}.txt", "r") as f:
            data = f.read().splitlines()
    except FileNotFoundError:
        print(f"File {dataset}.txt not found")
        continue

    speedlist = []
    forcelist = []

    for line in data:
        speedraw, forceraw = line.split(',')
        speedstr = speedraw.split(':')[1].strip()
        forcestr = forceraw.split(':')[1].strip()

        speed = float(speedstr)
        force = float(forcestr)

        speedlist.append(speed)
        forcelist.append(force)


    plt.plot(speedlist, forcelist, '-o', label=dataset)

plt.title("Propeller thrust vs PWM speed input")
plt.xlabel("Speed")
plt.ylabel("Force")
plt.grid()
plt.legend()

plt.savefig("loading/thrust_profile_plot.png")

plt.show()