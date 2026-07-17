import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.animation import FuncAnimation, PillowWriter

LOG_FILE = '/root/ros_workspace/test_logs/step_response_12.csv'
PDF_DIR = '/root/ros_workspace/test_plot_scripts/plots_pdf'
GIF_DIR = '/root/ros_workspace/test_plot_scripts/plots_gif'
MP4_DIR = '/root/ros_workspace/test_plot_scripts/plots_mp4'
SHOW_CONTROL_PLOTS = False
SAVE_PDF = False
SAVE_GIF = True
SAVE_MP4 = True
MAX_PLOT_TIME = 3.0

def real_gains(data):
    valid_data = data[(data['kp'] > 0) | (data['ki'] > 0) | (data['kd'] > 0)]
    if not valid_data.empty:
        kp_real = valid_data['kp'].iloc[0]
        ki_real = valid_data['ki'].iloc[0]
        kd_real = valid_data['kd'].iloc[0]
        return kp_real, ki_real, kd_real
    else:
        return 0.0, 0.0, 0.0

def calculate_euclidean_metrics(time, error):
    e0 = error.iloc[0]
    if e0 == 0:
        return 0.0, 0.0
    
    if e0 < 0:
        peak_val = error.max()
        overshoot = peak_val if peak_val > 0 else 0.0
    else:
        peak_val = error.min()
        overshoot = peak_val if peak_val < 0 else 0.0
        
    os_percent = abs(overshoot / e0) * 100.0

    threshold_tau = 0.368 * e0
    
    if e0 < 0:
        tau_mask = error >= threshold_tau
    else:
        tau_mask = error <= threshold_tau
        
    if tau_mask.any():
        tau = time[tau_mask].iloc[0] 
    else:
        tau = 0.0
        
    return os_percent, tau

def run_analysis_and_plot():
    raw_data = pd.read_csv(LOG_FILE)
    kp, ki, kd = real_gains(raw_data)

    mask = (raw_data['error_yaw'] != 0) | (raw_data['error_pitch'] != 0)
    if mask.any():
        first_idx = raw_data[mask].index[0]
        data = raw_data.iloc[first_idx:].copy()
        data['time'] = data['time'] - data['time'].iloc[0]
    else:
        data = raw_data.copy()
        print("No target detected in the log.")

    e_yaw_0 = data['error_yaw'].iloc[0]
    e_pitch_0 = data['error_pitch'].iloc[0]
    norm_0 = np.sqrt(e_yaw_0**2 + e_pitch_0**2)
    
    if norm_0 != 0:
        data['error_euclidean'] = - (data['error_yaw'] * e_yaw_0 + data['error_pitch'] * e_pitch_0) / norm_0
    else:
        data['error_euclidean'] = 0.0

    limit_time = MAX_PLOT_TIME if MAX_PLOT_TIME is not None else data['time'].iloc[-1]
    plot_data = data[data['time'] <= limit_time]
    
    total_frames = len(plot_data)
    duration_sec = plot_data['time'].iloc[-1]
    fps = total_frames / duration_sec if duration_sec > 0 else 10

    rows = 2 if SHOW_CONTROL_PLOTS else 1
    fig, axes = plt.subplots(rows, 1, figsize=(8, 4.5 * rows), sharex=True, squeeze=False) # x * 100, y * 100
    fig.suptitle(f"Analisi Risposta al Gradino a 4 metri di distanza (Risposta Combinata)\nKp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}", fontsize=14)
    
    os_eucl, tau_eucl = calculate_euclidean_metrics(data['time'], data['error_euclidean'])

    ax_err = axes[0, 0]
    line_err, = ax_err.plot([], [], label='Errore Vettoriale (Pixel)', color='purple', linewidth=2)
    ax_err.axhline(0, color='red', linestyle='--')
    ax_err.set_title("Risposta Combinata (Pan + Tilt)")
    ax_err.set_ylabel("Distanza dal Centro [Pixel]")
    ax_err.set_xlim(0, limit_time)
    
    y_min = data['error_euclidean'].min()
    y_max = data['error_euclidean'].max()
    padding = abs(y_max - y_min) * 0.1
    ax_err.set_ylim(y_min - padding, y_max + padding)

    if not SHOW_CONTROL_PLOTS:
        ax_err.set_xlabel("Time (s)")
    ax_err.grid(True)
    
    info_text = f'OS: {os_eucl:.1f}%\nTau (τ): {tau_eucl:.2f}s'
    ax_err.text(0.95, 0.05, info_text,
                transform=ax_err.transAxes, verticalalignment='bottom', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.5))
                    
    ax_err.axvline(x=tau_eucl, color='orange', linestyle=':', alpha=0.8, label='Tau (τ)')
    ax_err.legend()

    lines = [line_err]

    if SHOW_CONTROL_PLOTS:
        ax_ctrl = axes[1, 0]
        line_yaw, = ax_ctrl.plot([], [], color='cyan', label='Controllo Pan')
        line_pitch, = ax_ctrl.plot([], [], color='orange', label='Controllo Tilt')
        ax_ctrl.set_title("Segnali di Controllo (Output PID)")
        ax_ctrl.set_ylabel("Speed")
        ax_ctrl.set_xlabel("Time (s)")
        ax_ctrl.set_xlim(0, limit_time)
        
        c_min = min(data['control_yaw'].min(), data['control_pitch'].min())
        c_max = max(data['control_yaw'].max(), data['control_pitch'].max())
        c_padding = abs(c_max - c_min) * 0.1
        ax_ctrl.set_ylim(c_min - c_padding, c_max + c_padding)
        
        ax_ctrl.grid(True)
        ax_ctrl.legend()
        lines.extend([line_yaw, line_pitch])

    def update(frame):
        line_err.set_data(plot_data['time'].iloc[:frame], plot_data['error_euclidean'].iloc[:frame])
        if SHOW_CONTROL_PLOTS:
            line_yaw.set_data(plot_data['time'].iloc[:frame], plot_data['control_yaw'].iloc[:frame])
            line_pitch.set_data(plot_data['time'].iloc[:frame], plot_data['control_pitch'].iloc[:frame])
        return lines

    ani = FuncAnimation(fig, update, frames=total_frames, interval=1000/fps, blit=True, repeat=False)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if SAVE_PDF:
        os.makedirs(PDF_DIR, exist_ok=True)
        base_name = os.path.basename(LOG_FILE)
        name_no_ext = os.path.splitext(base_name)[0]
        pdf_filename = os.path.join(PDF_DIR, f"euclidean_{name_no_ext}.pdf")
        plt.savefig(pdf_filename, dpi=300)
    
    if SAVE_GIF:
        os.makedirs(GIF_DIR, exist_ok=True)
        base_name = os.path.basename(LOG_FILE)
        name_no_ext = os.path.splitext(base_name)[0]
        gif_filename = os.path.join(GIF_DIR, f"euclidean_{name_no_ext}.gif")
        ani.save(gif_filename, writer=PillowWriter(fps=fps))

    if SAVE_MP4:
        os.makedirs(MP4_DIR, exist_ok=True)
        base_name = os.path.basename(LOG_FILE)
        name_no_ext = os.path.splitext(base_name)[0]
        mp4_filename = os.path.join(MP4_DIR, f"euclidean_{name_no_ext}.mp4")
        ani.save(mp4_filename, writer='ffmpeg', fps=fps, dpi=120)
        
    plt.show()

if __name__ == "__main__":
    if os.path.exists(LOG_FILE):
        run_analysis_and_plot()
    else:
        print(f"Log file {LOG_FILE} does not exist.")