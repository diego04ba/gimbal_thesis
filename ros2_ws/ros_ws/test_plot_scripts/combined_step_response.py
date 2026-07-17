import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

LOG_FILES = [
    '/root/ros_workspace/test_logs/step_response_5.csv',
    '/root/ros_workspace/test_logs/step_response_13.csv',
    '/root/ros_workspace/test_logs/step_response_8.csv',
    '/root/ros_workspace/test_logs/step_response_11.csv',
    '/root/ros_workspace/test_logs/step_response_6.csv',
    '/root/ros_workspace/test_logs/step_response_4.csv',
    '/root/ros_workspace/test_logs/step_response_3.csv',
    '/root/ros_workspace/test_logs/step_response_10.csv',
    '/root/ros_workspace/test_logs/step_response_2.csv',
    '/root/ros_workspace/test_logs/step_response_9.csv',
    '/root/ros_workspace/test_logs/step_response_12.csv',
]

PDF_DIR = '/root/ros_workspace/test_plot_scripts/plots_pdf'
SAVE_PDF = True
MAX_PLOT_TIME = 3.0

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

def run_combined_analysis():
    fig, ax_err = plt.subplots(1, 1, figsize=(10, 5))
    fig.suptitle("Confronto Risposte al Gradino", fontsize=14)

    num_files = len(LOG_FILES)

    for i, file_path in enumerate(LOG_FILES):
        if not os.path.exists(file_path):
            print(f"Attenzione: Log file {file_path} not found . Skipping...")
            continue

        raw_data = pd.read_csv(file_path)

        mask = (raw_data['error_yaw'] != 0) | (raw_data['error_pitch'] != 0)
        if mask.any():
            first_idx = raw_data[mask].index[0]
            data = raw_data.iloc[first_idx:].copy()
            data['time'] = data['time'] - data['time'].iloc[0]
        else:
            print(f"No target detected in the log: {file_path}")
            continue

        e_yaw_0 = data['error_yaw'].iloc[0]
        e_pitch_0 = data['error_pitch'].iloc[0]
        norm_0 = np.sqrt(e_yaw_0**2 + e_pitch_0**2)
    
        if norm_0 != 0:
            data['error_euclidean'] = - (data['error_yaw'] * e_yaw_0 + data['error_pitch'] * e_pitch_0) / norm_0
        else:
            data['error_euclidean'] = 0.0

        current_alpha = max(0.2, 1.0 - (i * 0.3))
        current_lw = 2.5 if i == 0 else 1.5
        intensity = 0.9 if num_files <= 1 else 0.9 - (i * (0.6 / (num_files - 1)))
        current_color = plt.cm.Purples(intensity)

        file_name = os.path.splitext(os.path.basename(file_path))[0]
        ax_err.plot(data['time'], data['error_euclidean'], 
                    label=file_name, 
                    color=current_color, 
                    linewidth=current_lw, 
                    alpha=current_alpha)

    ax_err.axhline(0, color='red', linestyle='--')
    ax_err.set_title("Risposte Combinate Sovrapposte")
    ax_err.set_ylabel("Distanza dal Centro [Pixel]")
    ax_err.set_xlabel("Time (s)")
    ax_err.set_xlim(0, MAX_PLOT_TIME)
    ax_err.grid(True)
    # ax_err.legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if SAVE_PDF:
        os.makedirs(PDF_DIR, exist_ok=True)
        base_pdf_path = os.path.join(PDF_DIR, "combined_step_response.pdf")
        unique_pdf_path = get_unique_filename(base_pdf_path)
        plt.savefig(unique_pdf_path, dpi=300)
    plt.show()

if __name__ == "__main__":
    run_combined_analysis()