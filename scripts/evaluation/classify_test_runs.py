''' This script reads a journal file and filters out lines that begin with 'Created' or 'Opened' or <!> and then reads the remaining lines containing object in the RUNNING state into a pandas dataframe.
    The script then groups the rows based on closeness of timestamps (2 minutes) and creates new journal files for each group.
'''

import pandas as pd
import io
import re

filename = 'xxx.jnl'

lines=[]
#Filter lines that begin with 'Created' or 'Opened' or <!>
with open(filename) as f:
    for line in f:
        if "Created" in line:
            continue
        if "Opened" in line:
            continue
        if "<!>" in line:
            continue
        lines.append(line)

lines = [line.replace(';\n', '') for line in lines]
lines = [re.sub(r'([0-9]+\.[0-9]+)\:\s[0-9]+\.[0-9]+', r'\1', line) for line in lines]
buffer = io.StringIO("\n".join(lines))
df=pd.read_csv(buffer, sep=';', names=['timestamp','client_id','ip','x','y','z','heading_rad','speed_long_m_s','speed_lat_m_s','accel_long_m_s2','accel_lat_m_s2','drive_direction','obj_state','arm_readiness','error_status'])
df.drop_duplicates(subset=['timestamp'], keep='first', inplace=True)
df['timestamp'] = pd.to_datetime(df['timestamp'],unit='s')
df=df[df.obj_state.isin(['RUNNING'])]


#Group the rows based on closeness of timestamps (2 minutes)
df['Group'] = (df['timestamp'].diff().dt.total_seconds() > 2*60).cumsum()

# print start and end timestamp of each group as well as count and duration in time
print(df.groupby('Group').agg({'timestamp': ['first', 'last', 'count', lambda x: x.max() - x.min()]}))

# create new journal files for each group
for name, group in df.groupby('Group'):
    group.to_csv(f'group_{name}.jnl', sep=';', index=False)


import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# plot each group sequentially using seaborn with the title of each group (commented out for now)

'''
for name, group in df.groupby('Group'):
    #plot the x and y coordinates of each object showing heading with an arrow
    plt.figure()
    sns.scatterplot(x='x', y='y', hue='client_id', palette="muted", data=group)
    for i, row in group.iterrows():
        plt.arrow(row['x'], row['y'], np.cos(row['heading_rad']), np.sin(row['heading_rad']), head_width=0.1, head_length=0.1, fc='k', ec='k')
    plt.title(f'Group {name}')
    plt.show()

    # plot longitudinal speed for each of the 6 objects in the group
    plt.figure()
    sns.lineplot(x='timestamp', y='speed_long_m_s', hue='client_id', palette="muted", data=group)
    plt.title(f'Group {name}')
    plt.show()
    # plot lateral speed for each of the 6 objects in the group
    plt.figure()
    sns.lineplot(x='timestamp', y='speed_lat_m_s', hue='client_id', palette="muted", data=group)
    plt.title(f'Group {name}')
    plt.show()
'''