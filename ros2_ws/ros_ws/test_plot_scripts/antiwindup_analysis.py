import pandas as pd
import matplotlib.pyplot as plt
import os

LOG_DIR = '/root/ros_workspace/test_logs'
PDF_DIR = '/root/ros_workspace/test_plot_scripts/plots_pdf'
FILE_OFF = os.path.join(LOG_DIR, 'anti_windup_off_2.csv')
FILE_ON = os.path.join(LOG_DIR, 'anti_windup_on_2.csv')
SAVE_PDF = False  

def get_unique_filename(base_path):
    if not os.path.exists(base_path):
        return base_path
    
    directory, filename = os.path.split(base_path)
    name, ext = os.path.splitext(filename)
    counter = 1
    
    while True:
        new_filename = f"{name}_{counter}{ext}"
        new_path = os.path.join(directory, new_filename)
        if not os.path.exists(new_path):
            return new_path
        counter += 1

def load_and_preprocess(filepath):
    df = pd.read_csv(filepath)
    df['time'] = df['time'] - df['time'].iloc[0]
    return df

def plot_comparison():
    data_off = load_and_preprocess(FILE_OFF)
    data_on = load_and_preprocess(FILE_ON)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12), sharex=True)
    fig.suptitle('Confronto Anti-Windup: Tilt Axis (Saturazione)', fontsize=16, fontweight='bold')

    axes[0, 0].plot(data_off['time'], data_off['error_pitch'], color='red', label='Error (OFF)')
    axes[0, 0].axhline(0, color='red', linestyle='--')
    axes[0, 0].set_title("Tilt Error (Anti-Windup OFF)")
    axes[0, 0].grid(True)
    axes[0, 0].set_ylabel("Pixels")

    axes[0, 1].plot(data_on['time'], data_on['error_pitch'], color='green', label='Error (ON)')
    axes[0, 1].axhline(0, color='green', linestyle='--')
    axes[0, 1].set_title("Tilt Error (Anti-Windup ON)")
    axes[0, 1].grid(True)

    axes[1, 0].plot(data_off['time'], data_off['control_pitch'], color='orange')
    axes[1, 0].set_title("Control Signal (OFF)")
    axes[1, 0].grid(True)
    axes[1, 0].set_ylabel("Speed")

    axes[1, 1].plot(data_on['time'], data_on['control_pitch'], color='orange')
    axes[1, 1].set_title("Control Signal (ON)")
    axes[1, 1].grid(True)

    axes[2, 0].plot(data_off['time'], data_off['integral_pitch'], color='purple')
    axes[2, 0].set_title("Integral Term (OFF - Windup Visibile)")
    axes[2, 0].grid(True)
    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 0].set_ylabel("Sum")

    axes[2, 1].plot(data_on['time'], data_on['integral_pitch'], color='darkgreen')
    axes[2, 1].set_title("Integral Term (ON - Windup Limitato)")
    axes[2, 1].grid(True)
    axes[2, 1].set_xlabel("Time (s)")

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    if SAVE_PDF:
        os.makedirs(PDF_DIR, exist_ok=True)
        base_pdf_path = os.path.join(PDF_DIR, "confronto_antiwindup.pdf")
        unique_pdf_path = get_unique_filename(base_pdf_path)
        plt.savefig(unique_pdf_path, dpi=300)
    plt.show()

if __name__ == "__main__":
    if os.path.exists(FILE_OFF) and os.path.exists(FILE_ON):
        plot_comparison()
    else:
        print(f"Ensure that {FILE_OFF} and {FILE_ON} exist.")