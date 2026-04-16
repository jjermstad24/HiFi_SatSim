import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import os
import sys

def ecef_to_latlon(x, y, z):
    """
    Convert ECEF coordinates to Geodetic Latitude and Longitude using WGS84 model.
    """
    # WGS84 constants
    a = 6378137.0
    f = 1 / 298.257223563
    b = a * (1 - f)
    e2 = 1 - (b**2 / a**2)
    
    lon = np.degrees(np.arctan2(y, x))
    
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1 - e2)) # Initial guess
    
    # Iterate for better accuracy
    for _ in range(5):
        N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
        alt = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1 - e2 * (N / (N + alt))))
        
    return np.degrees(lat), lon

def main(parquet_path):
    if not os.path.exists(parquet_path):
        print(f"Error: {parquet_path} not found.")
        sys.exit(1)
        
    print(f"Loading data from {parquet_path}...")
    df = pd.read_parquet(parquet_path)
    
    # Extract vehicle position (ECI)
    print("Extracting position and rotation data...")
    try:
        r_eci = df[[
            'vehicle.dyn_body.core_body.state.trans.position[0]',
            'vehicle.dyn_body.core_body.state.trans.position[1]',
            'vehicle.dyn_body.core_body.state.trans.position[2]'
        ]].values
        
        # Extract rotation matrix (ECI to ECEF)
        # earth.planet.pfix.state.rot.T_parent_this is I2B where Earth is 'this'
        T = []
        for i in range(3):
            row = []
            for j in range(3):
                col_name = f'earth.planet.pfix.state.rot.T_parent_this[{i}][{j}]'
                row.append(df[col_name].values)
            T.append(row)
        T = np.array(T) # Shape (3, 3, N)
        T = np.transpose(T, (2, 1, 0)) # Shape (N, 3, 3) - Note: depends on if T is stored row-major or col-major.
        # Trick usually stores as T[row][col]. So i=row, j=col.
        # We want to multiply T * r_eci where r_eci is a column vector.
        # r_ecef[k] = Sum_j T[i][j] * r_eci[j]
        # Our T is (N, 3, 3) where T[n, i, j] is row i, col j of matrix at time n.
    except KeyError as e:
        print(f"Error: Missing column in parquet: {e}")
        sys.exit(1)
        
    print("Transforming vehicle position to ECEF...")
    r_ecef = np.einsum('nij,nj->ni', T, r_eci)
    
    print("Converting to Lat/Lon...")
    lats, lons = ecef_to_latlon(r_ecef[:,0], r_ecef[:,1], r_ecef[:,2])
    
    # Handle longitude jumps for plotting (e.g. at -180/180)
    diffs = np.diff(lons)
    jumps = np.where(np.abs(diffs) > 180)[0]
    lons_plot = np.insert(lons, jumps + 1, np.nan)
    lats_plot = np.insert(lats, jumps + 1, np.nan)

    # Extract SAR footprint and animation frames
    print("Extracting frames data...")
    sar_polygons_lons = []
    sar_polygons_lats = []
    
    # Determine frame indices based on sim_time to get a target of ~200-300 frames
    if 'sim_time' in df.columns:
        max_time = df['sim_time'].max()
        target_frames = 300
        desired_timestep = max(0.1, max_time / target_frames)
        print(f"Max sim_time is {max_time:.1f}s. Resampling animation at a ~{desired_timestep:.1f}s timestep.")
        
        frame_indices = []
        next_time = df['sim_time'].iloc[0]
        for i, t in enumerate(df['sim_time']):
            if t >= next_time:
                frame_indices.append(i)
                next_time += desired_timestep
    else:
        # Fallback if sim_time is missing
        step = max(1, len(df) // 300)
        frame_indices = list(range(0, len(df), step))

    poly_lons_list = []
    poly_lats_list = []
    sat_lons = [lons[idx] for idx in frame_indices]
    sat_lats = [lats[idx] for idx in frame_indices]
    
    if 'vehicle.sar.footprint_valid' in df.columns:
        valid_mask = df['vehicle.sar.footprint_valid'].values == 1
        for idx in frame_indices:
            if valid_mask[idx]:
                poly_lons = []
                poly_lats = []
                for i in range(4):
                    x = df[f'vehicle.sar.footprint_ecef[{i}][0]'].values[idx]
                    y = df[f'vehicle.sar.footprint_ecef[{i}][1]'].values[idx]
                    z = df[f'vehicle.sar.footprint_ecef[{i}][2]'].values[idx]
                    lat_i, lon_i = ecef_to_latlon(x, y, z)
                    poly_lons.append(lon_i)
                    poly_lats.append(lat_i)
                # Close the polygon
                poly_lons.append(poly_lons[0])
                poly_lats.append(poly_lats[0])
                
                poly_lons_list.append(poly_lons)
                poly_lats_list.append(poly_lats)
                sar_polygons_lons.append(poly_lons)
                sar_polygons_lats.append(poly_lats)
            else:
                poly_lons_list.append([])
                poly_lats_list.append([])
    else:
        poly_lons_list = [[] for _ in frame_indices]
        poly_lats_list = [[] for _ in frame_indices]

    # Visualization - Matplotlib

    print("Generating Matplotlib plot...")
    plt.figure(figsize=(15, 8))
    plt.plot(lons_plot, lats_plot, 'b-', linewidth=1.5, label='Ground Track')
    
    # Plot start and end
    plt.plot(lons[0], lats[0], 'go', label='Start')
    plt.plot(lons[-1], lats[-1], 'ro', label='End')

    # Plot SAR footprints
    if sar_polygons_lons:
        plt.plot(sar_polygons_lons[0], sar_polygons_lats[0], 'r-', alpha=0.5, linewidth=0.5, label='SAR Footprint')
        for i in range(1, len(sar_polygons_lons)):
            plt.plot(sar_polygons_lons[i], sar_polygons_lats[i], 'r-', alpha=0.5, linewidth=0.5)

    # Extract and plot targets
    target_lats = []
    target_lons = []
    for t in range(1, 100): # Check up to 100 targets
        base = f'target{t}'
        if f'{base}.pos_inertial[0]' in df.columns:
            r_target_eci = df[[f'{base}.pos_inertial[0]', f'{base}.pos_inertial[1]', f'{base}.pos_inertial[2]']].values
            r_target_ecef = np.einsum('nij,nj->ni', T, r_target_eci)
            t_lats, t_lons = ecef_to_latlon(r_target_ecef[:,0], r_target_ecef[:,1], r_target_ecef[:,2])
            target_lats.append(t_lats[0])
            target_lons.append(t_lons[0])
            plt.text(t_lons[0], t_lats[0], f'T{t}', fontsize=8, color='darkorange')
        else:
            if t > 5: break

    if target_lats:
        plt.scatter(target_lons, target_lats, color='darkorange', marker='x', s=30, label='Targets')

    plt.xlabel('Longitude (deg)')
    plt.ylabel('Latitude (deg)')
    plt.title(f'Satellite Ground Track - {os.path.basename(os.path.dirname(parquet_path))}')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.xlim(-180, 180)
    plt.ylim(-90, 90)
    plt.legend()
    plt.tight_layout()
    plt.savefig('ground_track.png', dpi=150)
    print("Saved ground_track.png")
    
    # Visualization - Plotly Animation
    print("Generating animated Plotly map...")
    fig = go.Figure()
    
    # Ground Track (trace 0)
    fig.add_trace(go.Scattergeo(
        lon = lons_plot,
        lat = lats_plot,
        mode = 'lines',
        line = dict(width = 2, color = 'blue'),
        name = 'Ground Track'
    ))
    
    # Targets (trace 1 if targets exist)
    if target_lats:
        fig.add_trace(go.Scattergeo(
            lon = target_lons,
            lat = target_lats,
            mode = 'markers+text',
            marker = dict(size = 8, color = 'darkorange', symbol = 'x'),
            text = [f'T{i+1}' for i in range(len(target_lats))],
            textposition = 'top center',
            name = 'Targets'
        ))

    dynamic_trace_start = len(fig.data)
    
    # Satellite Marker
    fig.add_trace(go.Scattergeo(
        lon = [sat_lons[0]],
        lat = [sat_lats[0]],
        mode = 'markers',
        marker = dict(size = 8, color = 'red', symbol='square'),
        name = 'Satellite'
    ))
    
    # SAR Footprint Polygon
    fig.add_trace(go.Scattergeo(
        lon = poly_lons_list[0] if poly_lons_list else [],
        lat = poly_lats_list[0] if poly_lats_list else [],
        mode = 'lines',
        line = dict(width = 2, color = 'rgba(255, 0, 0, 0.8)'),
        fill = 'toself',
        fillcolor = 'rgba(255, 0, 0, 0.3)',
        name = 'SAR Footprint'
    ))

    # Frames
    frames = []
    for k in range(len(frame_indices)):
        frames.append(go.Frame(
            data=[
                go.Scattergeo(lon=[sat_lons[k]], lat=[sat_lats[k]]),
                go.Scattergeo(lon=poly_lons_list[k], lat=poly_lats_list[k])
            ],
            name=f'frame{k}',
            traces=[dynamic_trace_start, dynamic_trace_start + 1]
        ))
        
    fig.frames = frames

    # Play button and slider
    fig.update_layout(
        title = 'Satellite Ground Track Over Time',
        geo = dict(
            showland = True,
            landcolor = "rgb(243, 243, 243)",
            countrycolor = "rgb(204, 204, 204)",
            showcountries = True,
            projection_type = 'equirectangular'
        ),
        margin={"r":0,"t":50,"l":0,"b":0},
        updatemenus=[dict(
            type="buttons",
            buttons=[dict(
                label="Play",
                method="animate",
                args=[None, {"frame": {"duration": 50, "redraw": True}, "fromcurrent": True, "transition": {"duration": 0}}]
            ), dict(
                label="Pause",
                method="animate",
                args=[[None], {"frame": {"duration": 0, "redraw": False}, "mode": "immediate", "transition": {"duration": 0}}]
            )]
        )],
        sliders=[dict(
            steps=[dict(
                method='animate',
                args=[[f'frame{k}'], {"mode": "immediate", "frame": {"duration": 50, "redraw": True}, "transition": {"duration": 0}}],
                label=f'{k}'
            ) for k in range(len(frame_indices))],
            transition=dict(duration=0),
            x=0,
            y=0,
            currentvalue=dict(font=dict(size=12), visible=True, xanchor='center'),
            len=1.0
        )]
    )

    fig.write_html('ground_track.html')
    print("Saved ground_track.html")

if __name__ == "__main__":
    default_path = "simulation/RUN_nominal/simulation_log.parquet"
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = default_path
        
    main(path)
