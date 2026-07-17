import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

LOG_DIR = '/root/ros_workspace/test_logs'
PDF_DIR = '/root/ros_workspace/test_plot_scripts/plots_pdf'
GIF_DIR = '/root/ros_workspace/test_plot_scripts/plots_gif'
MP4_DIR = '/root/ros_workspace/test_plot_scripts/plots_mp4'
FILE_OFF = os.path.join(LOG_DIR, 'anti_windup_off_3.csv')
FILE_ON = os.path.join(LOG_DIR, 'anti_windup_on_3.csv')
SAVE_PDF = False
SAVE_GIF = False  
SAVE_MP4 = True
MAX_PLOT_TIME = 32.26

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
    mask = df['error_pitch'] != 0
    if mask.any():
        offset = df.loc[mask, 'time'].iloc[0] - 2.0
        df['time'] = df['time'] - offset
    else:
        df['time'] = df['time'] - df['time'].iloc[0]
    
    df = df[df['time'] >= 0].reset_index(drop=True)
    
    if not df.empty and df['time'].iloc[0] > 0:
        first_row = df.iloc[[0]].copy()
        first_row['time'] = 0.0
        df = pd.concat([first_row, df]).reset_index(drop=True)
        
    return df

def get_padded_bounds(s1, s2):
    val_min = min(s1.min(), s2.min())
    val_max = max(s1.max(), s2.max())
    pad = abs(val_max - val_min) * 0.1
    if pad == 0:
        pad = 1.0
    return val_min - pad, val_max + pad

def plot_comparison():
    data_off_full = load_and_preprocess(FILE_OFF)
    data_on_full = load_and_preprocess(FILE_ON)

    limit_time = MAX_PLOT_TIME if MAX_PLOT_TIME is not None else max(data_off_full['time'].iloc[-1], data_on_full['time'].iloc[-1])
    
    data_off = data_off_full[data_off_full['time'] <= limit_time]
    data_on = data_on_full[data_on_full['time'] <= limit_time]

    fig, axes = plt.subplots(3, 2, figsize=(8, 9), sharex=True, sharey="row")
    fig.suptitle('Confronto Anti-Windup: Tilt Axis (Saturazione)', fontsize=16, fontweight='bold')

    e_min, e_max = get_padded_bounds(data_off['error_pitch'], data_on['error_pitch'])
    c_min, c_max = get_padded_bounds(data_off['control_pitch'], data_on['control_pitch'])
    i_min, i_max = get_padded_bounds(data_off['integral_pitch'], data_on['integral_pitch'])

    line_err_off, = axes[0, 0].plot([], [], color='red', label='Error (OFF)', linewidth=2)
    axes[0, 0].axhline(0, color='red', linestyle='--')
    axes[0, 0].set_title("Tilt Error (Anti-Windup OFF)")
    axes[0, 0].grid(True)
    axes[0, 0].set_ylabel("Pixels")
    axes[0, 0].set_xlim(0, limit_time)
    axes[0, 0].set_ylim(e_min, e_max)

    line_err_on, = axes[0, 1].plot([], [], color='green', label='Error (ON)', linewidth=2)
    axes[0, 1].axhline(0, color='green', linestyle='--')
    axes[0, 1].set_title("Tilt Error (Anti-Windup ON)")
    axes[0, 1].grid(True)
    axes[0, 1].set_xlim(0, limit_time)
    axes[0, 1].set_ylim(e_min, e_max)

    line_ctrl_off, = axes[1, 0].plot([], [], color='orange', linewidth=2)
    axes[1, 0].set_title("Control Signal (OFF)")
    axes[1, 0].grid(True)
    axes[1, 0].set_ylabel("Speed")
    axes[1, 0].set_xlim(0, limit_time)
    axes[1, 0].set_ylim(c_min, c_max)

    line_ctrl_on, = axes[1, 1].plot([], [], color='orange', linewidth=2)
    axes[1, 1].set_title("Control Signal (ON)")
    axes[1, 1].grid(True)
    axes[1, 1].set_xlim(0, limit_time)
    axes[1, 1].set_ylim(c_min, c_max)

    line_int_off, = axes[2, 0].plot([], [], color='purple', linewidth=2)
    axes[2, 0].set_title("Integral Term (OFF - Windup Visibile)")
    axes[2, 0].grid(True)
    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 0].set_ylabel("Sum")
    axes[2, 0].set_xlim(0, limit_time)
    axes[2, 0].set_ylim(i_min, i_max)

    line_int_on, = axes[2, 1].plot([], [], color='darkgreen', linewidth=2)
    axes[2, 1].set_title("Integral Term (ON - Windup Limitato)")
    axes[2, 1].grid(True)
    axes[2, 1].set_xlabel("Time (s)")
    axes[2, 1].set_xlim(0, limit_time)
    axes[2, 1].set_ylim(i_min, i_max)

    fps = 20 
    total_frames = int((limit_time * 2) * fps)

    lines = [line_err_off, line_err_on, line_ctrl_off, line_ctrl_on, line_int_off, line_int_on]

    def update(frame):
        current_time = frame / fps
        
        if current_time <= limit_time:
            time_off = current_time
            time_on = 0.0
        else:
            time_off = limit_time
            time_on = current_time - limit_time
        
        idx_off = np.searchsorted(data_off['time'], time_off)
        idx_on = np.searchsorted(data_on['time'], time_on)
        
        line_err_off.set_data(data_off['time'].iloc[:idx_off], data_off['error_pitch'].iloc[:idx_off])
        line_err_on.set_data(data_on['time'].iloc[:idx_on], data_on['error_pitch'].iloc[:idx_on])
        
        line_ctrl_off.set_data(data_off['time'].iloc[:idx_off], data_off['control_pitch'].iloc[:idx_off])
        line_ctrl_on.set_data(data_on['time'].iloc[:idx_on], data_on['control_pitch'].iloc[:idx_on])
        
        line_int_off.set_data(data_off['time'].iloc[:idx_off], data_off['integral_pitch'].iloc[:idx_off])
        line_int_on.set_data(data_on['time'].iloc[:idx_on], data_on['integral_pitch'].iloc[:idx_on])
        
        return lines

    ani = FuncAnimation(fig, update, frames=total_frames, interval=1000/fps, blit=True, repeat=False)

    plt.tight_layout(rect=[0, 0, 1, 0.96], h_pad=4.0, w_pad=3.0)
    
    if SAVE_PDF:
        os.makedirs(PDF_DIR, exist_ok=True)
        base_pdf_path = os.path.join(PDF_DIR, "confronto_antiwindup.pdf")
        unique_pdf_path = get_unique_filename(base_pdf_path)
        plt.savefig(unique_pdf_path, dpi=300)
    
    if SAVE_GIF:
        os.makedirs(GIF_DIR, exist_ok=True)
        base_gif_path = os.path.join(GIF_DIR, "confronto_antiwindup.gif")
        unique_gif_path = get_unique_filename(base_gif_path)
        ani.save(unique_gif_path, writer=PillowWriter(fps=fps))
    
    if SAVE_MP4:
        os.makedirs(MP4_DIR, exist_ok=True)
        base_mp4_path = os.path.join(MP4_DIR, "confronto_antiwindup.mp4")
        unique_mp4_path = get_unique_filename(base_mp4_path)
        ani.save(unique_mp4_path, writer='ffmpeg', fps=fps, dpi=120)
        
    plt.show()

if __name__ == "__main__":
    if os.path.exists(FILE_OFF) and os.path.exists(FILE_ON):
        plot_comparison()
    else:
        print(f"Ensure that {FILE_OFF} and {FILE_ON} exist.")