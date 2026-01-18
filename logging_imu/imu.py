# Author: Per-Ivar Faust

import os
# Suppress Qt portal warnings
# os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'

import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
import numpy as np

# %% Load IMU data
def load_imu_data(file_path):
    """
    Load IMU data from a CSV file.

    Parameters:
    file_path (str): Path to the CSV file containing IMU data.

    Returns:
    pd.DataFrame: DataFrame containing the IMU data.
    """
    imu_data = pd.read_csv(file_path)
    return imu_data

# %% Visualize IMU vectors in 3D
def visualize_imu_vectors_3d(imu_data):
    """
    Create an interactive 3D visualization of IMU vectors (acceleration, gyroscope, magnetometer).
    
    Parameters:
    imu_data (pd.DataFrame): DataFrame containing IMU data with columns aX, aY, aZ, gX, gY, gZ, mX, mY, mZ
    """
    # Create figure
    fig = go.Figure()
    
    # Calculate scaling for better visualization (normalize vectors)
    a_scale = 1.0 / 1000  # Acceleration scaling
    g_scale = 1.0  # Gyroscope scaling
    m_scale = 1.0 / 50  # Magnetometer scaling
    
    # Create frames for animation
    frames = []
    for i in range(len(imu_data)):
        # Acceleration vector (blue)
        ax, ay, az = imu_data.iloc[i][['aX', 'aY', 'aZ']] * a_scale
        # Gyroscope vector (red)
        gx, gy, gz = imu_data.iloc[i][['gX', 'gY', 'gZ']] * g_scale
        # Magnetometer vector (green)
        mx, my, mz = imu_data.iloc[i][['mX', 'mY', 'mZ']] * m_scale
        
        frame_data = [
            # Acceleration vector
            go.Scatter3d(
                x=[0, ax], y=[0, ay], z=[0, az],
                mode='lines+markers',
                line=dict(color='blue', width=6),
                marker=dict(size=[0, 8]),
                name='Akselerasjon (a)',
                showlegend=(i==0)
            ),
            go.Cone(
                x=[ax], y=[ay], z=[az],
                u=[ax*0.1], v=[ay*0.1], w=[az*0.1],
                colorscale='Blues',
                showscale=False,
                sizemode='absolute',
                sizeref=0.3
            ),
            # Gyroscope vector
            go.Scatter3d(
                x=[0, gx], y=[0, gy], z=[0, gz],
                mode='lines+markers',
                line=dict(color='red', width=6),
                marker=dict(size=[0, 8]),
                name='Gyroskop (g)',
                showlegend=(i==0)
            ),
            go.Cone(
                x=[gx], y=[gy], z=[gz],
                u=[gx*0.1], v=[gy*0.1], w=[gz*0.1],
                colorscale='Reds',
                showscale=False,
                sizemode='absolute',
                sizeref=0.3
            ),
            # Magnetometer vector
            go.Scatter3d(
                x=[0, mx], y=[0, my], z=[0, mz],
                mode='lines+markers',
                line=dict(color='green', width=6),
                marker=dict(size=[0, 8]),
                name='Magnetometer (m)',
                showlegend=(i==0)
            ),
            go.Cone(
                x=[mx], y=[my], z=[mz],
                u=[mx*0.1], v=[my*0.1], w=[mz*0.1],
                colorscale='Greens',
                showscale=False,
                sizemode='absolute',
                sizeref=0.3
            ),
        ]
        
        frames.append(go.Frame(
            data=frame_data,
            name=str(i),
            layout=go.Layout(
                title_text=f"Tidspunkt: {i} - {imu_data.iloc[i]['rtcTime']}"
            )
        ))
    
    # Add initial frame
    fig.add_traces(frames[0].data)
    
    # Add slider and play/pause buttons
    fig.update_layout(
        title=f"IMU Vektorer i 3D - Tidspunkt: 0 - {imu_data.iloc[0]['rtcTime']}",
        scene=dict(
            xaxis=dict(title='X', range=[-2, 2]),
            yaxis=dict(title='Y', range=[-2, 2]),
            zaxis=dict(title='Z', range=[-2, 2]),
            aspectmode='cube'
        ),
        updatemenus=[
            dict(
                type="buttons",
                buttons=[
                    dict(label="▶ Play",
                         method="animate",
                         args=[None, {"frame": {"duration": 100, "redraw": True},
                                     "fromcurrent": True,
                                     "mode": "immediate"}]),
                    dict(label="⏸ Pause",
                         method="animate",
                         args=[[None], {"frame": {"duration": 0, "redraw": False},
                                       "mode": "immediate",
                                       "transition": {"duration": 0}}])
                ],
                x=0.1, y=0, xanchor="left", yanchor="top"
            )
        ],
        sliders=[{
            "active": 0,
            "yanchor": "top",
            "y": 0,
            "xanchor": "left",
            "x": 0.1,
            "currentvalue": {
                "prefix": "Tidspunkt: ",
                "visible": True,
                "xanchor": "right"
            },
            "pad": {"b": 10, "t": 50},
            "len": 0.9,
            "steps": [
                {
                    "args": [[f.name], {
                        "frame": {"duration": 0, "redraw": True},
                        "mode": "immediate",
                        "transition": {"duration": 0}
                    }],
                    "label": str(i),
                    "method": "animate"
                }
                for i, f in enumerate(frames)
            ]
        }]
    )
    
    fig.frames = frames
    fig.show()

# %%


if __name__ == "__main__":

    imu_file_dir = Path('logfiles')

    # Print current working directory
    print("Current working directory:", Path.cwd())

    if not imu_file_dir.exists():
        raise FileNotFoundError(f"The directory {imu_file_dir} does not exist.")


    imu_file = "dataLog00000.TXT"
    imu_data = load_imu_data(imu_file_dir.joinpath(imu_file))
    
    # Drop last col
    imu_data = imu_data.iloc[:, :-1]
    
    
    print(imu_data.head())
    
    # Visualize IMU vectors in 3D
    visualize_imu_vectors_3d(imu_data)

