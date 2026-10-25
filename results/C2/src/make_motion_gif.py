"""Combine the twelve FreeCAD-rendered frames; requires Pillow only."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
root=Path(__file__).resolve().parents[1]
frames=[]
for i in range(12):
    frame=Image.open(root/'images'/f'motion-{i:02d}.png').convert('RGB')
    frame.thumbnail((960,660),Image.Resampling.LANCZOS)
    page=Image.new('RGB',(960,700),'white');page.paste(frame,(0,0))
    draw=ImageDraw.Draw(page)
    draw.text((18,674),f'C2 | cyclic 3 deg | mast {i*30:03d} deg | solved CAD positions, slow display',fill=(30,50,70))
    frames.append(page)
frames[0].save(root/'images/motion.gif',save_all=True,append_images=frames[1:],duration=220,loop=0,disposal=2)
with Image.open(root/'images/motion.gif') as check:assert check.n_frames==12
print('Verified 12-frame motion.gif')
