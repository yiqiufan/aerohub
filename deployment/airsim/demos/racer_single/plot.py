"""Offline, read-only flight data visualization; writes figures only."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path('/result')
samples=json.loads((root/'trajectory-samples.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(11,4.5),constrained_layout=True)
colors={'arming':'gray','takeoff':'#e0a020','tracking':'#176ca4','stopping':'#9c4db0','landing':'#43964a'}
for phase in dict.fromkeys(s['phase'] for s in samples):
    rows=[s for s in samples if s['phase']==phase]
    axes[0].plot([s['east'] for s in rows],[s['north'] for s in rows],'.-',
                 markersize=2,lw=1,color=colors.get(phase),label=phase)
    axes[1].plot([s['wall_seconds'] for s in rows],[s['height'] for s in rows],'.-',
                 markersize=2,lw=1,color=colors.get(phase),label=phase)
axes[0].scatter([0],[0],s=45,c='black',marker='x',label='start')
axes[0].set(xlabel='East from start (m)',ylabel='North from start (m)',title='Observed AirSim physics trajectory')
axes[0].axis('equal')
axes[1].set(xlabel='Wall seconds since preflight ready',ylabel='Height above start (m)',title='Flight and landing observations')
axes[1].axhline(0,color='gray',lw=.6)
for ax in axes:ax.grid(alpha=.25)
axes[0].legend(fontsize=8)
fig.suptitle('Drone0 native RACER short exploration\nSampled evidence; not full-map coverage or collision-event validation',fontsize=11)
fig.savefig(root/'trajectory.png',dpi=170)
fig.savefig(root/'trajectory.pdf')
