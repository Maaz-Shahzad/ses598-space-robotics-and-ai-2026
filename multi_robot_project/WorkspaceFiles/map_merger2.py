import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.cluster.hierarchy import fcluster, linkage
import os

import pandas as pd
import numpy as np
from scipy.cluster.hierarchy import fcluster, linkage
import matplotlib.pyplot as plt

def merge_maps_refined(file1, file2):
    # 1. GROUND TRUTH SPAWN POINTS (From your launch file)
    # Robot 1 spawns at x=-1.5, y=-3.0
    # Robot 2 spawns at x=1.5, y=-3.0
    R1_OFFSET = {'x': -1.5, 'y': -3.0, 'yaw': 1.57}
    R2_OFFSET = {'x': 1.5, 'y': -3.0, 'yaw': 1.57}

    LABELS = ['person','car',"truck", "stop sign", "cat_block", "table"]
    
    # Cluster distances tuned for "straight line" noise
    DIST_MAP = {'person': 3.0, 'stop sign': 3.0, 'car': 6.0, 'truck': 8.5,'default': 2.0}

    # Load Data
    df1 = pd.read_csv(file1)
    df2 = pd.read_csv(file2)

#    def transform_to_world(df, spawn):
#        # We add the spawn yaw to the robot's relative odom yaw
#        total_yaw = df['robot_yaw'] + 1.5708
#        
#        world_x = spawn['x'] + df['robot_x'] + (df['cam_z'] * np.cos(total_yaw) - df['cam_x'] * np.sin(total_yaw))
#        world_y = spawn['y'] + df['robot_y'] + (df['cam_z'] * np.sin(total_yaw) + df['cam_x'] * np.cos(total_yaw))
#
#        # world_x = spawn['x'] - df['robot_y'] - df['cam_x'] * np.sin(total_yaw)
#        # world_y = spawn['y'] + df['robot_x'] + df['cam_x'] * np.cos(total_yaw)
#        return world_x, world_y


#    def transform_to_world(df, spawn):
#        # Spawn Yaw is 90 degrees (1.5708 rad)
#        S_YAW = 1.5708 
#        
#        # 1. Map Robot Odom to World Displacement
#        # We rotate the (robot_x, robot_y) by the initial spawn heading
#        # This turns "Robot Forward" into "World North"
#        odom_world_x = df['robot_x'] * np.cos(S_YAW) - df['robot_y'] * np.sin(S_YAW)
#        odom_world_y = df['robot_x'] * np.sin(S_YAW) + df['robot_y'] * np.cos(S_YAW)
#
#        # 2. Map Camera Detection to World displacement
#        # This requires the total heading (Spawn + Current Odom Yaw)
#        total_yaw = df['robot_yaw'] + S_YAW
#        
#        # x = forward (depth), y = lateral (left)
#        obj_rel_x = df['cam_z']
#        obj_rel_y = -df['cam_x'] 
#
#        obj_world_x = obj_rel_x * np.cos(total_yaw) - obj_rel_y * np.sin(total_yaw)
#        obj_world_y = obj_rel_x * np.sin(total_yaw) + obj_rel_y * np.cos(total_yaw)
#
#        # 3. Combine everything
#        # World Position = Spawn point + Robot's world movement + Object's world offset
#        world_x = spawn['y'] + odom_world_x + obj_world_x
#        world_y = spawn['x'] + odom_world_y + obj_world_y
#        
#        return world_x, world_y



#    def transform_to_world(df, spawn):
#        """
#        Convert odom-aligned camera detections into world coordinates.
#        """
#
#        # Spawn offset (world origin of odom frame)
#        x0 = spawn['x']
#        y0 = spawn['y']
#
#        # Robot world position (corrected for spawn offset)
#        xr = df['robot_x'].values + x0
#        yr = df['robot_y'].values + y0
#
#        # Dynamic yaw from dataframe (IMPORTANT FIX)
#        theta = df['robot_yaw'].values  # MUST exist per timestep
#
#        c = np.cos(theta)
#        s = np.sin(theta)
#
#        # Camera == odom frame (no extra rotation)
#        x_local = df['cam_x'].values   # forward
#        y_local = df['cam_y'].values   # left
#
#        # Rotate into world frame
#        x_world = xr + c * x_local - s * y_local
#        y_world = yr + s * x_local + c * y_local
#        return world_x, world_y
#
    def transform_to_world(df, spawn):
        """
        Wx, Wy: World frame        
        Rix, Riy: Robot initialization frame (odom frame origin)
        Rcx, Rcy: Robot current frame (rotated from Rix by robot_yaw and translated by robot odom x and y)
        cam_x, cam_y: Object coordinates in Rcx, Rcy frame
        """
        theta = df['robot_yaw']
        Rcx = df['robot_x']
        Rcy = df['robot_y']
        Rix = Rcx + df['cam_x']*np.cos(theta) - df['cam_y']*np.sin(theta)
        Riy = Rcy + df['cam_x']*np.sin(theta) + df['cam_y']*np.cos(theta)
        Wx = - Riy - spawn['x']
        Wy = Rix - spawn['y']
        return Wx, Wy

    # Apply transformations
    df1['world_x'], df1['world_y'] = transform_to_world(df1, R1_OFFSET)
    df2['world_x'], df2['world_y'] = transform_to_world(df2, R2_OFFSET)

    # Merge and Filter
    df = pd.concat([df1, df2], ignore_index=True)
    df['label'] = df['label'].fillna(df['label'])
    df = df[df['label'].isin(LABELS)].copy()

    final_results = []

    # 3. CLUSTERING BY LABEL
    for label in df['label'].unique():
        label_df = df[df['label'] == label].copy()
        this_dist = DIST_MAP.get(label, DIST_MAP['default'])
        
        coords = label_df[['world_x', 'world_y']].values
        if len(coords) > 1:
            Z = linkage(coords, method='centroid')
            clusters = fcluster(Z, this_dist, criterion='distance')
            label_df['cluster'] = clusters
        else:
            label_df['cluster'] = 1

        clustered = label_df.groupby('cluster').agg({
            'world_x': 'mean', 'world_y': 'mean', 'label': 'count'
        }).rename(columns={'label': 'detections'})
        
        for _, row in clustered.iterrows():
            final_results.append({
                'label': label, 
                'x': round(row['world_x'], 2), 
                'y': round(row['world_y'], 2),
                'detections': int(row['detections'])
            })

    final_map = pd.DataFrame(final_results)
    final_map.to_csv('final_map.csv', index=False)
    print("\n--- Final Refined Map ---")
    print(final_map.to_string(index=False))
    return final_map

def plot_map(input_csv='final_map.csv', output_png='final_world_map.png'):
    if not os.path.exists(input_csv):
        print(f"Error: {input_csv} not found. Run the merger script first.")
        return

    # 1. Load the merged data
    df = pd.read_csv(input_csv)

    # 2. Define colors for common labels
    color_map = {
        'person': 'tab:red',
        'car': 'tab:blue',
        'truck': 'tab:green',
        'stop sign': 'tab:orange',
        'bicycle': 'tab:purple'
    }

    # 3. Create the scatter plot
    # Note: We avoid plt.figure() as per specific environment guidelines
    for label, group in df.groupby('label'):
        color = color_map.get(label, 'tab:gray')
        plt.scatter(group['x'], group['y'], label=label, c=color, s=100, edgecolors='black', alpha=0.8)
        
        # Add text labels next to the points
        for _, row in group.iterrows():
            plt.text(row['x'] + 0.2, row['y'] + 0.2, label, fontsize=9, fontweight='bold')

    # 4. Formatting
    plt.xlabel('World X Coordinate (m)')
    plt.ylabel('World Y Coordinate (m)')
    plt.title('Aggregated Object Map (Robot 1 & 2)')
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # Ensure the plot is square (1m on X = 1m on Y)
    plt.gca().set_aspect('equal', adjustable='datalim')
    
    plt.tight_layout()

    # 5. Save the result
    plt.savefig(output_png, dpi=300)
    print(f"Plot saved successfully as {output_png}")

if __name__ == "__main__":
    merge_maps_refined('robot1_map_data.csv', 'robot2_map_data.csv')
    plot_map()
#    plot_map(input_csv="original_ground_truth.csv", output_png="original_ground_truth.png")




















