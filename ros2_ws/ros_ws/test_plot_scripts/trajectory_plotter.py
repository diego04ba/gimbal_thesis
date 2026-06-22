import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.image as mpimg
import os

LOG_FILE = '/root/ros_workspace/test_logs/multi_tracking_1.csv'
IMG_DIR = '/root/ros_workspace/test_plot_scripts/marker_id'
PDF_DIR = '/root/ros_workspace/test_plot_scripts/plots_pdf'
ERROR_THRESHOLD = 5.0 # pixels
STABILITY_TIME = 0.5 # seconds
SAVE_PDF = False

zoom_level = 0.02  
y_offset = 4.0

def find_marker_positions(df):
    markers = {}
    dt = df['time'].diff().mean()
    if np.isnan(dt) or dt == 0: dt = 0.05
    min_samples = int(STABILITY_TIME / dt)
    grouped = df[df['target_id'] != -1].groupby('target_id')
    
    for marker_id, group in grouped:
        is_stable = (abs(group['error_yaw']) < ERROR_THRESHOLD) & \
                    (abs(group['error_pitch']) < ERROR_THRESHOLD)
        
        is_stable_window = is_stable.rolling(window=min_samples).sum() == min_samples
        
        if is_stable_window.any():
            stable_segment = group[is_stable_window]
            avg_yaw = stable_segment['angle_yaw'].mean()
            avg_pitch = stable_segment['angle_pitch'].mean()
            markers[marker_id] = (avg_yaw, avg_pitch)
    return markers

def plot_with_markers(filepath):
    if not os.path.exists(filepath):
        print(f"Error: File {filepath} not found.")
        return

    data = pd.read_csv(filepath)
    data['time'] = data['time'] - data['time'].iloc[0]

    marker_positions = find_marker_positions(data)
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Linea gradiente
    x, y, t = data['angle_yaw'].values, data['angle_pitch'].values, data['time'].values
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    norm = plt.Normalize(t.min(), t.max())
    lc = LineCollection(segments, cmap='viridis', norm=norm, linewidth=2)
    lc.set_array(t)
    ax.add_collection(lc)
    
    for m_id, (pos_x, pos_y) in marker_positions.items():
        img_name = f"marker_id_{int(m_id)}.png"
        img_path = os.path.join(IMG_DIR, img_name)
        
        if os.path.exists(img_path):
            img = mpimg.imread(img_path)
            imagebox = OffsetImage(img, zoom=zoom_level)
            ab = AnnotationBbox(imagebox, (pos_x, pos_y), frameon=False)
            ax.add_artist(ab)
            ax.text(pos_x, pos_y - (y_offset + 4), f"ID {int(m_id)}", ha='center', fontsize=8, fontweight='bold')
        else:
            ax.scatter(pos_x, pos_y, color='red', marker='*')
            ax.text(pos_x, pos_y + 3, f"ID {int(m_id)}", ha='center', fontsize=8)

    ax.set_xlim(0, 360)
    ax.set_ylim(-90, 0)
    ax.set_xlabel("Pan [Degrees]")
    ax.set_ylabel("Tilt [Degrees]")
    ax.set_title("Traiettoria e Posizione Marker")
    ax.grid(True, linestyle='--', alpha=0.6)
    
    if SAVE_PDF:
        os.makedirs(PDF_DIR, exist_ok=True)
        base_name = os.path.basename(filepath)
        name_no_ext = os.path.splitext(base_name)[0]
        pdf_filename = os.path.join(PDF_DIR, f"trajectory_{name_no_ext}.pdf")
        plt.savefig(pdf_filename, dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    if os.path.exists(LOG_FILE):
        plot_with_markers(LOG_FILE)
    else:
        print(f"Log file {LOG_FILE} does not exist.")