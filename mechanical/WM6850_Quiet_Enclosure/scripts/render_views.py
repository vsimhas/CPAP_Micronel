"""Technical previews rendered from CAD tessellation, not an illustrative mock-up."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageDraw, ImageFont
import FreeCAD as App
import Part, MeshPart

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output'
data=json.loads((OUT/'preview_meshes.json').read_text())
doc=App.openDocument(str(ROOT/'cad'/'WM6850_Quiet_Enclosure.FCStd'))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11})
light=np.array([-.4,-.7,1.0]);light=light/np.linalg.norm(light)

def render(mode):
    allpolys=[]; allcolours=[]
    for d in data:
        name=d['name']
        if mode=='cutaway' and name[:2] in ['02','03','04','05']: continue
        verts=np.array(d['vertices']);tri=np.array(d['triangles'])
        if name.startswith('01') and mode=='cutaway':
            obj=next(o for o in doc.Objects if o.Label==name.replace('_',' '))
            s=obj.Shape.copy()
            s=s.cut(Part.makeBox(230,6,100,App.Vector(-1,-1,14)))
            s=s.cut(Part.makeBox(6,164,100,App.Vector(215,-1,14)))
            m=MeshPart.meshFromShape(Shape=s,LinearDeflection=.25,AngularDeflection=.3,Relative=False)
            vv,tt=m.Topology
            verts=np.array([[v.x,v.y,v.z] for v in vv]);tri=np.array(tt)
        if mode=='exploded':
            dz={'02':50,'03':65,'04':78,'05':88,'06':20,'07':10,'08':45,'09':45,'10':20}.get(name[:2],0)
            verts[:,2]+=dz
            if name[:2] in ['11','12','13']:verts[:,0]-={'11':35,'12':23,'13':29}[name[:2]]
        polys=verts[tri]
        n=np.cross(polys[:,1]-polys[:,0],polys[:,2]-polys[:,0])
        n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12)
        shade=.62+.38*np.abs(n@light)
        colours=np.minimum(np.array(d['colour'])[None,:]*shade[:,None],1)
        allpolys.extend(polys);allcolours.extend(colours)
    # Per-pixel z buffer avoids painter-sort artefacts in long CAD triangles.
    polygons=np.array(allpolys)
    az,el=np.radians([-52,29])
    toward=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
    right=np.array([-np.sin(az),np.cos(az),0]);up=np.cross(toward,right)
    projected=polygons@np.stack([right,up,toward],axis=1)
    lo=projected[:,:,:2].min(axis=(0,1));hi=projected[:,:,:2].max(axis=(0,1))
    width,height=2200,1500
    scale=min(1900/(hi[0]-lo[0]),1110/(hi[1]-lo[1]))
    projected[:,:,0]=(projected[:,:,0]-(lo[0]+hi[0])/2)*scale+width/2
    projected[:,:,1]=-(projected[:,:,1]-(lo[1]+hi[1])/2)*scale+800
    pixels=np.empty((height,width,3),dtype=np.uint8);pixels[:]=[244,247,248]
    depth=np.full((height,width),-np.inf)
    for tri,col in zip(projected,allcolours):
        x0,y0,z0=tri[0];x1,y1,z1=tri[1];x2,y2,z2=tri[2]
        den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
        if abs(den)<1e-9:continue
        left=max(0,int(np.floor(tri[:,0].min())));right_px=min(width-1,int(np.ceil(tri[:,0].max())))
        lower=max(0,int(np.floor(tri[:,1].min())));upper=min(height-1,int(np.ceil(tri[:,1].max())))
        if right_px<left or upper<lower:continue
        yy,xx=np.mgrid[lower:upper+1,left:right_px+1]
        a=((y1-y2)*(xx-x2)+(x2-x1)*(yy-y2))/den
        b=((y2-y0)*(xx-x2)+(x0-x2)*(yy-y2))/den;c=1-a-b
        zz=a*z0+b*z1+c*z2
        patch=depth[lower:upper+1,left:right_px+1]
        mask=(a>=-1e-7)&(b>=-1e-7)&(c>=-1e-7)&(zz>patch)
        patch[mask]=zz[mask]
        pixels[lower:upper+1,left:right_px+1][mask]=(np.array(col)*255).astype(np.uint8)
    img=Image.fromarray(pixels);draw=ImageDraw.Draw(img)
    regular=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',30)
    bold=ImageFont.truetype('C:/Windows/Fonts/segoeuib.ttf',52)
    small=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',24)
    draw.text((105,55),{'cutaway':'Inside the quiet enclosure','assembled':'Assembled enclosure','exploded':'Assembly arrangement'}[mode],
             font=bold,fill='#163643')
    sub={'cutaway':'Lid removed; front and right walls cut away for visibility.',
         'assembled':'220 x 160 mm body  |  118 mm overall height including feet and mass plate',
         'exploded':'Components separated vertically for clarity; hardware omitted.'}[mode]
    draw.text((105,130),sub,font=regular,fill='#4e6772')
    draw.text((105,1410),'REV A  /  BENCH PROTOTYPE  /  SUPPLIER BLOWER SHAPE APPROXIMATE',font=small,fill='#6e7e86')
    img.save(OUT/(mode+'.png'))
for mode in ['cutaway','assembled','exploded']:render(mode)
print('Rendered three CAD previews')
