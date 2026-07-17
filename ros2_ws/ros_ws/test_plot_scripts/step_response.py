import pandas as pd
import matplotlib.pyplot as plt
import os

LOG_FILE = '/root/ros_workspace/test_logs/step_response_11.csv'
PDF_DIR = '/root/ros_workspace/test_plot_scripts/plots_pdf'
SHOW_CONTROL_PLOTS = False
SAVE_PDF = True
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

def calculate_metrics(time, error):
    e0 = error.iloc[0]
    if e0 == 0:
        return 0.0, 0.0
        
    abs_err = error.abs()
    
    diff = abs_err.diff()
    growing_mask = diff > 0
    
    if growing_mask.any():
        first_bounce_idx = growing_mask.idxmax()
        
        tail_error = abs_err.loc[first_bounce_idx:]
        max_dev = tail_error.max()
    else:
        max_dev = 0.0
        
    os_percent = (max_dev / abs(e0)) * 100.0

    threshold_tau = 0.368 * abs(e0)
    tau_mask = abs_err <= threshold_tau
    
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
        if data['error_yaw'].iloc[0] > 0:
            data['error_yaw'] = -data['error_yaw']
            data['control_yaw'] = -data['control_yaw']

        if data['error_pitch'].iloc[0] > 0:
            data['error_pitch'] = -data['error_pitch']
            data['control_pitch'] = -data['control_pitch']
    else:
        data = raw_data
        print("No target detected in the log.")

    rows = 2 if SHOW_CONTROL_PLOTS else 1
    fig, axes = plt.subplots(rows, 2, figsize=(12, 4 * rows), sharex=True, squeeze=False)
    fig.suptitle(f"Analisi Risposta al Gradino a 2 metri di distanza\nKp={kp}, Ki={ki}, Kd={kd}", fontsize=14)
    os_yaw, tau_yaw = calculate_metrics(data['time'], data['error_yaw'])
    os_pitch, tau_pitch = calculate_metrics(data['time'], data['error_pitch'])

    axes[0, 0].plot(data['time'], data['error_yaw'], label='Errore Pan', color='blue')
    axes[0, 0].axhline(0, color='red', linestyle='--')
    axes[0, 0].set_title("Risposta Pan")
    axes[0, 0].set_xlim(0, MAX_PLOT_TIME)
    axes[0, 0].grid(True)
    info_text_yaw = f'OS: {os_yaw:.1f}%\nTau (τ): {tau_yaw:.2f}s'
    axes[0, 0].text(0.95, 0.05, info_text_yaw,
                    transform=axes[0, 0].transAxes, verticalalignment='bottom', horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.5))
    axes[0, 0].axvline(x=tau_yaw, color='orange', linestyle=':', alpha=0.8, label='Tau (τ)')
    axes[0, 0].legend()

    axes[0, 1].plot(data['time'], data['error_pitch'], label='Errore Tilt', color='green')
    axes[0, 1].axhline(0, color='red', linestyle='--')
    axes[0, 1].set_title("Risposta Tilt")
    axes[0, 1].set_xlim(0, MAX_PLOT_TIME)
    axes[0, 1].grid(True)
    info_text_pitch = f'OS: {os_pitch:.1f}%\nTau (τ): {tau_pitch:.2f}s'
    axes[0, 1].text(0.95, 0.05, info_text_pitch,
                    transform=axes[0, 1].transAxes, verticalalignment='bottom', horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.5))
    axes[0, 1].axvline(x=tau_pitch, color='orange', linestyle=':', alpha=0.8, label='Tau (τ)')
    axes[0, 1].legend()

    if SHOW_CONTROL_PLOTS:
        axes[1, 0].plot(data['time'], data['control_yaw'], color='cyan')
        axes[1, 0].set_title("Controllo Pan (Output PID)")
        axes[1, 0].set_xlim(0, MAX_PLOT_TIME)
        axes[1, 0].grid(True)

        axes[1, 1].plot(data['time'], data['control_pitch'], color='orange')
        axes[1, 1].set_title("Controllo Tilt (Output PID)")
        axes[1, 1].set_xlim(0, MAX_PLOT_TIME)
        axes[1, 1].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if SAVE_PDF:
        os.makedirs(PDF_DIR, exist_ok=True)
        base_name = os.path.basename(LOG_FILE)
        name_no_ext = os.path.splitext(base_name)[0]
        pdf_filename = os.path.join(PDF_DIR, f"{name_no_ext}.pdf")
        plt.savefig(pdf_filename, dpi=300)
    plt.show()

if __name__ == "__main__":
    if os.path.exists(LOG_FILE):
        run_analysis_and_plot()
    else:
        print(f"Log file {LOG_FILE} does not exist.")