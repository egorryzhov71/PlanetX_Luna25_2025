import os
import matplotlib.pyplot as plt

# --- Helper function to ensure the save directory exists ---
def ensure_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

# --- Data Loading Functions ---
def load_data(file_path):
    """
    Reads a data file (CSV with header: time, mass, altitude, speed)
    and returns lists for time, mass (in tonnes), altitude, and speed.
    """
    time_list = []
    mass_list = []
    altitude_list = []
    speed_list = []
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.read().strip().splitlines()
    # Skip header and process non-empty lines
    for line in lines[1:]:
        if not line.strip():
            continue
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 4:
            continue
        time_list.append(float(parts[0]))
        mass_list.append(float(parts[1]) / 1000)  # convert kg to tonnes
        altitude_list.append(float(parts[2]))
        speed_list.append(float(parts[3]))
    return time_list, mass_list, altitude_list, speed_list

# --- Global Data ---
# Load KSP data
time_ksp, massa_ksp, height_ksp, speed_ksp = load_data("./data/KSP_Stats.txt")
# Load Math model data
time_mm, massa_mm, height_mm, speed_mm = load_data("./data/MathModel_Stats.txt")

# Colors to use in graphs
ksp_color    = "#008000"  # Green for KSP
mm_color     = "#ff0000"  # Red for Math model
massa_color  = "#ff0000"  # For error in mass
height_color = "#008000"  # For error in height
speed_color  = "#0000ff"  # For error in speed

# Directory to save graphs
save_dir = "Files/Data/Graphs_KSP"
ensure_directory(save_dir)

# --- Graphing Functions ---
def Save(name):
    plt.legend()
    save_path = os.path.join(save_dir, f"{name}.png")
    plt.savefig(save_path)
    plt.cla()  # clear current axes
    print(f"Saved graph: {save_path}")

def GraphTimeMassa():
    plt.xlabel("Время, с")
    plt.ylabel("Масса, т")
    plt.plot(time_ksp, massa_ksp, label='KSP', color=ksp_color, linewidth=1)
    plt.plot(time_mm, massa_mm, label='Math model', color=mm_color, linewidth=1)
    Save("TimeMassa")

def GraphTimeHeight():
    plt.xlabel("Время, с")
    plt.ylabel("Высота, м")
    plt.plot(time_ksp, height_ksp, label='KSP', color=ksp_color, linewidth=1)
    plt.plot(time_mm, height_mm, label='Math model', color=mm_color, linewidth=1)
    Save("TimeHeight")

def GraphTimeSpeed():
    plt.xlabel("Время, с")
    plt.ylabel("Скорость, м/c")
    plt.plot(time_ksp, speed_ksp, label='KSP', color=ksp_color, linewidth=1)
    plt.plot(time_mm, speed_mm, label='Math model', color=mm_color, linewidth=1)
    Save("TimeSpeed")

def GraphHeightSpeed():
    plt.xlabel("Высота, м")
    plt.ylabel("Скорость, м/c")
    plt.plot(height_ksp, speed_ksp, label='KSP', color=ksp_color, linewidth=1)
    plt.plot(height_mm, speed_mm, label='Math model', color=mm_color, linewidth=1)
    Save("HeightSpeed")

def ErrorRate():
    plt.title("Относительная погрешность")
    plt.xlabel("Время, с")
    plt.ylabel("Погрешность, %")
    # Use the shorter time array for the common range.
    common_time = time_ksp if len(time_ksp) <= len(time_mm) else time_mm
    error_rate_massa = []
    error_rate_height = []
    error_rate_speed = []
    # Start calculation from index 10 (to skip startup noise)
    for i in range(10, len(common_time)):
        # Avoid division by zero (assume model data is nonzero)
        err_mass = abs((massa_ksp[i] - massa_mm[i]) / massa_mm[i] * 100) if massa_mm[i] != 0 else 0
        err_height = abs((height_ksp[i] - height_mm[i]) / height_mm[i] * 100) if height_mm[i] != 0 else 0
        err_speed = abs((speed_ksp[i] - speed_mm[i]) / speed_mm[i] * 100) if speed_mm[i] != 0 else 0
        error_rate_massa.append(err_mass)
        error_rate_height.append(err_height)
        error_rate_speed.append(err_speed)
    # Plot errors using the corresponding portion of the common time array
    plt.plot(common_time[10:], error_rate_massa, label='Масса', color=massa_color, linewidth=1)
    plt.plot(common_time[10:], error_rate_height, label='Высота', color=height_color, linewidth=1)
    plt.plot(common_time[10:], error_rate_speed, label='Скорость', color=speed_color, linewidth=1)
    Save("ErrorRate")

# --- Run Graphing Functions ---
GraphTimeMassa()
GraphTimeHeight()
GraphTimeSpeed()
GraphHeightSpeed()
ErrorRate()

print("Графики сохранены в ./data/Graphs_KSP")
