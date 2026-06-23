import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

LOG_FILE = '/root/ros_workspace/test_logs/step_response_5.csv'
PDF_DIR = '/root/ros_workspace/test_plot_scripts/plots_pdf'
SHOW_CONTROL_PLOTS = False
SAVE_PDF = False

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
        return 0.0, 0.0, 0.0
        
    max_idx = error.idxmax()
    
    after_max_error = error.loc[max_idx:]
    peak = after_max_error.min() if len(after_max_error) > 1 else 0.0
    
    os_percent = (peak / e0) * 100.0 if e0 != 0 else 0.0
    
    threshold_ts = 0.02 * e0
    out_of_band = time[error < threshold_ts]
    ts = out_of_band.iloc[-1] if not out_of_band.empty else time.iloc[-1]

    threshold_tau = 0.368 * e0
    tau_mask = error >= threshold_tau
    if tau_mask.any():
        tau = time[tau_mask].iloc[0] 
    else:
        tau = ts / 4.0 
        
    return os_percent, ts, tau

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

    data['error_euclidean'] = -np.sqrt(data['error_yaw']**2 + data['error_pitch']**2)

    rows = 2 if SHOW_CONTROL_PLOTS else 1
    fig, axes = plt.subplots(rows, 1, figsize=(10, 5 * rows), sharex=True, squeeze=False)
    fig.suptitle(f"Analisi Risposta al Gradino (Euclidea)\nKp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}", fontsize=14)
    
    os_eucl, ts_eucl, tau_eucl = calculate_euclidean_metrics(data['time'], data['error_euclidean'])

    ax_err = axes[0, 0]
    ax_err.plot(data['time'], data['error_euclidean'], label='Errore Vettoriale (Pixel)', color='purple', linewidth=2)
    ax_err.axhline(0, color='red', linestyle='--')
    ax_err.set_title("Risposta Combinata (Pan + Tilt)")
    ax_err.set_ylabel("Distanza dal Centro [Pixel]")
    if not SHOW_CONTROL_PLOTS:
        ax_err.set_xlabel("Time (s)")
    ax_err.grid(True)
    
    info_text = f'OS: {os_eucl:.1f}%\nTs(2%): {ts_eucl:.2f}s\nTau (τ): {tau_eucl:.2f}s'
    ax_err.text(0.95, 0.05, info_text,
                transform=ax_err.transAxes, verticalalignment='bottom', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.5))
                    
    ax_err.axvline(x=ts_eucl, color='blue', linestyle='--', alpha=0.5, label='Ts (2%)')
    ax_err.axvline(x=tau_eucl, color='orange', linestyle=':', alpha=0.8, label='Tau (τ)')
    ax_err.legend()

    if SHOW_CONTROL_PLOTS:
        ax_ctrl = axes[1, 0]
        ax_ctrl.plot(data['time'], data['control_yaw'], color='cyan', label='Controllo Pan')
        ax_ctrl.plot(data['time'], data['control_pitch'], color='orange', label='Controllo Tilt')
        ax_ctrl.set_title("Segnali di Controllo (Output PID)")
        ax_ctrl.set_ylabel("Speed")
        ax_ctrl.set_xlabel("Time (s)")
        ax_ctrl.grid(True)
        ax_ctrl.legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if SAVE_PDF:
        os.makedirs(PDF_DIR, exist_ok=True)
        base_name = os.path.basename(LOG_FILE)
        name_no_ext = os.path.splitext(base_name)[0]
        pdf_filename = os.path.join(PDF_DIR, f"euclidean_inv_{name_no_ext}.pdf")
        plt.savefig(pdf_filename, dpi=300)
        
    plt.show()

if __name__ == "__main__":
    if os.path.exists(LOG_FILE):
        run_analysis_and_plot()
    else:
        print(f"Log file {LOG_FILE} does not exist.")