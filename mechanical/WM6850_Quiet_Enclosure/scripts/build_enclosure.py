"""Rebuild the millimetre-based CAD package with FreeCAD's bundled Python.

Source-controlled parameters are in ../parameters.json. PROVISIONAL values
require measurement of the actual blower/purchased components before printing.
No SolidWorks installation is required to generate STEP geometry.
"""
from pathlib import Path
import json, math, csv, itertools
import FreeCAD as App
import Part, MeshPart, Import

ROOT = Path(__file__).resolve().parents[1]
P = json.loads((ROOT / 'parameters.json').read_text())
CAD = ROOT / 'cad'
OUT = ROOT / 'output'
for d in [CAD, OUT, CAD/'step_parts', CAD/'stl_print']:
    d.mkdir(parents=True, exist_ok=True)
V = App.Vector
doc = App.newDocument('WM6850_Quiet_Enclosure_RevA')
assembly = doc.addObject('App::Part', 'EnclosureAssembly')
assembly.Label = 'WM6850 quiet enclosure - Rev A BENCH PROTOTYPE'
parts = []
meshdata = []

def box(x,y,z,dx,dy,dz):
    return Part.makeBox(dx,dy,dz,V(x,y,z))

def cyl(r,h,x,y,z,axis=(0,0,1)):
    return Part.makeCylinder(r,h,V(x,y,z),V(*axis))

def ring(ro,ri,h,x,y,z,axis=(0,0,1)):
    return cyl(ro,h,x,y,z,axis).cut(cyl(ri,h+2,x-axis[0],y-axis[1],z-axis[2],axis))

def rounded(x,y,z,dx,dy,dz,r):
    s=box(x,y,z,dx,dy,dz)
    edges=[e for e in s.Edges if abs(e.Vertexes[0].Point.z-e.Vertexes[-1].Point.z)>dz-0.01]
    return s.makeFillet(r,edges)

def unite(shapes):
    s=shapes[0]
    for a in shapes[1:]: s=s.fuse(a)
    return s.removeSplitter()

def add(name,shape,kind,material,colour,notes='',printable=False):
    shape=shape.removeSplitter()
    if shape.isNull() or not shape.isValid() or len(shape.Solids)!=1:
        raise ValueError(f'{name}: expected one valid solid, got {len(shape.Solids)}')
    obj=doc.addObject('PartDesign::Feature',name)
    obj.Label=name.replace('_',' ')
    obj.Shape=shape
    assembly.addObject(obj)
    for prop,value in [('Category',kind),('MaterialSpecification',material),('DesignNotes',notes)]:
        obj.addProperty('App::PropertyString',prop,'Design')
        setattr(obj,prop,value)
    obj.addProperty('App::PropertyColor','PreviewColour','Design')
    obj.PreviewColour=colour
    if obj.ViewObject:
        obj.ViewObject.ShapeColor=colour
    bb=shape.BoundBox
    info={'name':name,'category':kind,'material':material,'notes':notes,'printable':printable,
          'volume_mm3':shape.Volume,'bbox_mm':[bb.XMin,bb.YMin,bb.ZMin,bb.XMax,bb.YMax,bb.ZMax],
          'colour':colour,'object':obj}
    parts.append(info)
    return obj

L,W,H=P['box_length'],P['box_width'],P['base_height']
t,f=P['wall'],P['floor']
lid_z=H+P['lid_gasket_compressed']
cx,cy=P['blower_centre']
bottom=P['blower_mount_plane_z']-2.0
top=bottom+P['blower_overall_height']
ox=cx+P['outlet_x_offset_PROVISIONAL']
oz=top-P['outlet_axis_from_top']
oy=cy+P['outlet_y_tip_from_centre']
gray=(0.32,0.43,0.52); teal=(0.08,0.53,0.58); soft=(0.82,0.39,0.18)

# One-piece open-top box. Baffles join opposite side walls and the floor.
shell=rounded(0,0,0,L,W,H,P['corner_radius']).cut(
    rounded(t,t,f,L-2*t,W-2*t,H+2,P['corner_radius']-t))
bx1,bx2=P['baffle_x']; bt=P['baffle_thickness']; gap=P['baffle_turn_gap']
shell=unite([shell,box(bx1,t,f,bt,W-2*t-gap,H-f),
             box(bx2,t+gap,f,bt,W-2*t-gap,H-f)])
for x,y in P['lid_fasteners']:
    shell=shell.fuse(cyl(6,H-f,x,y,f))
    shell=shell.cut(cyl(P['m3_insert_pilot_PROVISIONAL']/2,P['insert_depth']+0.1,x,y,H-P['insert_depth']))
for x,y in P['isolator_centres']:
    shell=shell.fuse(cyl(8,6,x,y,f))
    shell=shell.cut(cyl(P['m3_insert_pilot_PROVISIONAL']/2,6.1,x,y,4))
# Intake filter mounting bosses along x, four M3 insert pockets.
for y,z in [(8,16),(60,16),(8,98),(60,98)]:
    shell=shell.fuse(cyl(5,9,0,y,z,(1,0,0)))
    shell=shell.cut(cyl(P['m3_insert_pilot_PROVISIONAL']/2,6.1,-0.1,y,z,(1,0,0)))
iy0,iy1=P['inlet_window_y']; iz0,iz1=P['inlet_window_z']
shell=shell.cut(box(-1,iy0,iz0,t+2,iy1-iy0,iz1-iz0))
shell=shell.cut(cyl(P['outlet_wall_hole']/2,t+2,ox,W-t-1,oz,(0,1,0)))
# Rear cable exit: supplied two-piece gland enables routing preterminated leads.
shell=shell.cut(cyl(8,t+2,190,-1,30,(0,1,0)))
base=add('01_Base_with_intake_labyrinth',shell,'PRINT','PETG or ASA: bench-fit use only',gray,
         '4 mm walls/floor; 40 mm alternating baffle gaps; support side openings; no exposed foam.',True)

# Flat silicone gasket spans outer rim and both baffle tops (no acoustic shortcuts).
gasket=rounded(0,0,H,L,W,1,P['corner_radius']).cut(rounded(t,t,H-1,L-2*t,W-2*t,3,P['corner_radius']-t))
gasket=unite([gasket,box(bx1,t,H,bt,W-2*t-gap,1),box(bx2,t+gap,H,bt,W-2*t-gap,1)])
for x,y in P['lid_fasteners']:
    gasket=gasket.fuse(cyl(6,1,x,y,H))
    gasket=gasket.cut(cyl(1.75,3,x,y,H-1))
add('02_Rim_and_baffle_gasket',gasket,'CUT ELASTOMER','Nonporous silicone, 1.5 mm free / 1 mm fitted',soft,
    'CAD shows compressed thickness. Select actual material for emissions, cleanability and compression set.')
lid=rounded(0,0,lid_z,L,W,P['lid_thickness'],P['corner_radius'])
for x,y in P['lid_fasteners']: lid=lid.cut(cyl(1.75,6,x,y,lid_z-1))
add('03_Removable_lid',lid,'PRINT','PETG or ASA: bench-fit use only',gray,
    'Six M3 screws; tighten evenly to compress rim and baffle gasket without warping.',True)
# External damping sandwich: adhesive and metal do not contact inlet air.
add('04_External_damping_layer',rounded(30,25,109,160,110,1,5),'CUT SHEET',
    'Viscoelastic damping sheet, external only',(0.25,0.26,0.28),'No adhesive or damping material exposed to the air chamber.')
add('05_External_mass_plate',rounded(30,25,110,160,110,2,5),'CUT METAL',
    '2 mm aluminium, deburred',(0.65,0.69,0.72),'160 x 110 x 2 mm; bond over external damping layer.')

# Isolated carrier: every rigid blower attachment terminates on this plate.
plate=rounded(104,43,20,100,90,4,5)
plate=plate.cut(cyl(11,6,cx,cy,19))
mountpts=[]
for a in [P['blower_mount_angle_PROVISIONAL']+120*i for i in range(3)]:
    x=cx+P['blower_mount_pcd_PROVISIONAL']/2*math.cos(math.radians(a))
    y=cy+P['blower_mount_pcd_PROVISIONAL']/2*math.sin(math.radians(a))
    mountpts.append((x,y))
    plate=plate.fuse(ring(4.5,1.75,6,x,y,24))
    plate=plate.cut(cyl(1.75,12,x,y,19))
for x,y in P['isolator_centres']: plate=plate.cut(cyl(1.75,6,x,y,19))
add('06_Isolated_blower_carrier',plate,'PRINT','PETG or ASA: bench-fit use only',teal,
    '3 x M3 on assumed diameter 50 PCD at 36/156/276 deg; measure hole pattern and thread depth first.',True)
for i,(x,y) in enumerate(P['isolator_centres'],1):
    add(f'07_Isolator_{i}',cyl(6,10,x,y,10),'BUY REFERENCE','M3 bonded elastomer, nominal diameter 12 x 10',soft,
        'Elastomer envelope only; studs/nuts not modelled. Select for actual supported mass and deflection.')

# Supplier envelope, deliberately not a reverse-engineered production blower.
motor=cyl(28,15,cx,cy,30)
body=cyl(P['blower_body_diameter']/2,20,cx,cy,45)
cap=Part.makeCone(P['blower_body_diameter']/2,15,top-65,V(cx,cy,65))
blower=unite([motor,body,cap,cyl(7.5,2,cx,cy,28),
              cyl(P['outlet_barrel_od']/2,oy-(cy+20),ox,cy+20,oz,(0,1,0)),
              cyl(12,1.5,ox,oy-1.5,oz,(0,1,0))])
blower=blower.cut(cyl(P['outlet_bore_PROVISIONAL']/2,oy-cy+2,ox,cy,oz,(0,1,0)))
# Top central opening is visual only: datasheet does not fully define the intake.
blower=blower.cut(cyl(7.1,top-63,cx,cy,64))
for x,y in mountpts: blower=blower.cut(cyl(1.5,5,x,y,29.9))
add('08_WM6850_SUPPLIER_ENVELOPE_ONLY',blower,'REFERENCE','Supplier blower: 150 g',
    (0.70,0.73,0.77),'Nominal envelope. Volute, intake, wire exits and outlet bore are approximate; replace with supplier STEP before release.')

# Flexible sleeve has a relieved installed-ID around the blower retaining lip.
sy,ey=P['outlet_sleeve_start_y'],P['outlet_sleeve_end_y']
sleeve=ring(14,11,ey-sy,ox,sy,oz,(0,1,0))
sleeve=sleeve.cut(cyl(12.05,2.5,ox,oy-2,oz,(0,1,0)))
add('09_Flexible_outlet_sleeve',sleeve,'BUY REFERENCE','22 ID / 28 OD silicone hose, 66 mm nominal',soft,
    'Installed shape approximated at lip. Verify grip and retention on 21.9 OD barrel / 24 OD lip. End is plain hose, not a certified 22 mm connector.')
grommet=unite([ring(17,14,4,ox,156,oz,(0,1,0)),ring(20,14,2,ox,154,oz,(0,1,0)),ring(20,14,2,ox,160,oz,(0,1,0))])
add('10_Outlet_wall_grommet',grommet,'MOULD ELASTOMER','Soft silicone, 34 panel hole / 28 bore',soft,
    '4 mm panel groove. Custom envelope; verify compression and avoid hose collapse.')

# Intake cassette retains a flat, specified filter; mesh is an envelope only.
frame=box(-6,4,12,5,64,90).cut(box(-7,iy0,iz0,7,iy1-iy0,iz1-iz0))
frame=frame.cut(box(-3,12,22,4,44,70))
for z in [46,66]: frame=frame.fuse(box(-6,iy0,z,2,iy1-iy0,3))
for y,z in [(8,16),(60,16),(8,98),(60,98)]:frame=frame.cut(cyl(1.75,7,-7,y,z,(1,0,0)))
add('11_Intake_filter_retainer',frame,'PRINT','PETG or ASA: bench-fit use only',teal,
    '44 x 70 x 2 mm filter allowance, 36 x 62 clear window less two 3 mm grille bars; choose actual media before use.',True)
fg=box(-1,4,12,1,64,90).cut(box(-2,iy0,iz0,3,iy1-iy0,iz1-iz0))
for y,z in [(8,16),(60,16),(8,98),(60,98)]:fg=fg.cut(cyl(1.75,3,-2,y,z,(1,0,0)))
add('12_Filter_frame_gasket',fg,'CUT ELASTOMER','Nonporous silicone, 1 mm compressed',soft)
add('13_Filter_MEDIA_ENVELOPE',box(-3,12,22,2,44,70),'BUY REFERENCE','Specified CPAP intake media, supplier TBD',
    (0.85,0.85,0.73),'Solid CAD placeholder represents porous media, not an obstruction or printable part; filter edge seal required.')

# Split cable gland; purchased connectors pass before the two halves are fitted.
cable_gland=unite([ring(8,4.5,4,190,0,30,(0,1,0)),ring(11,4.5,2,190,-2,30,(0,1,0)),ring(11,4.5,2,190,4,30,(0,1,0))])
for idx,(xmin,dx) in enumerate([(178.0,12.0),(190.0,12.0)],1):
    add(f'14_Split_cable_gland_half_{idx}',cable_gland.common(box(xmin,-3,18,dx,10,24)),
        'MOULD ELASTOMER','Soft silicone split gland, 9 mm cable',soft,'Connector clearance and actual cable OD require measurement; add external strain relief.')
for i,(x,y) in enumerate([(15,15),(205,15),(15,145),(205,145)],1):
    add(f'15_External_foot_{i}',cyl(9,6,x,y,-6),'BUY REFERENCE','Adhesive elastomer foot, 18 diameter x 6',soft)

# Dimension register persists inside the editable FreeCAD file.
sheet=doc.addObject('Spreadsheet::Sheet','DimensionRegister')
sheet.set('A1','EDIT parameters.json AND REBUILD - mm unless stated')
sheet.setColumnWidth('A',330);sheet.setColumnWidth('B',380)
for i,(key,val) in enumerate(P.items(),2):
    sheet.set('A'+str(i),key);sheet.set('B'+str(i),str(val))
doc.recompute()
doc.saveAs(str(CAD/'WM6850_Quiet_Enclosure.FCStd'))
Import.export([assembly],str(CAD/'WM6850_Quiet_Enclosure.step'))
for info in parts:
    obj=info['object']
    # Global coordinates retained for assembly insertion at origin in SolidWorks.
    Part.export([obj],str(CAD/'step_parts'/(info['name']+'.step')))
    mesh=MeshPart.meshFromShape(Shape=obj.Shape,LinearDeflection=0.22,AngularDeflection=0.3,Relative=False)
    verts,facets=mesh.Topology
    meshdata.append({'name':info['name'],'colour':info['colour'],'category':info['category'],
                     'vertices':[[v.x,v.y,v.z] for v in verts],'triangles':facets})
    if info['printable']:
        sh=obj.Shape.copy()
        if info['name'].startswith('11_'):
            sh.rotate(V(0,0,0),V(0,1,0),90)
        bb=sh.BoundBox;sh.translate(V(-bb.XMin,-bb.YMin,-bb.ZMin))
        m=MeshPart.meshFromShape(Shape=sh,LinearDeflection=0.08,AngularDeflection=0.15,Relative=False)
        if not m.isSolid():raise ValueError('Non-closed STL '+info['name'])
        m.write(str(CAD/'stl_print'/(info['name']+'.stl')))

# Actual geometric checks, including collisions and STEP round-trip.
collisions=[]
for a,b in itertools.combinations(parts,2):
    sa,sb=a['object'].Shape,b['object'].Shape
    if sa.BoundBox.intersect(sb.BoundBox):
        vol=sa.common(sb).Volume
        if vol>0.01:collisions.append({'a':a['name'],'b':b['name'],'volume_mm3':round(vol,5)})
rt=Part.Shape();rt.read(str(CAD/'WM6850_Quiet_Enclosure.step'))
report={'parts':len(parts),'valid_single_solids':all(a['object'].Shape.isValid() and len(a['object'].Shape.Solids)==1 for a in parts),
        'step_roundtrip_valid':rt.isValid(),'step_roundtrip_solids':len(rt.Solids),
        'step_roundtrip_volume_mm3':rt.Volume,'sum_part_volume_mm3':sum(a['volume_mm3'] for a in parts),
        'interferences_over_0_01_mm3':collisions,'blower_top_z_mm':top,'lid_clearance_above_reference_mm':lid_z-top,
        'minimum_labyrinth_channel_area_mm2':(bx1-t)*(H-f),
        'inlet_grille_geometric_free_area_mm2':(iy1-iy0)*(iz1-iz0-6),
        'limitations':['No acoustic simulation or physical noise test','No CFD or measured filter pressure loss','No thermal qualification',
                       'Supplier envelope and mounting pattern require fit check','SolidWorks unavailable: native import/conversion untested']}
(OUT/'geometry_validation.json').write_text(json.dumps(report,indent=2))
(OUT/'preview_meshes.json').write_text(json.dumps(meshdata,separators=(',',':')))
manifest=[{k:v for k,v in a.items() if k!='object'} for a in parts]
(OUT/'parts_manifest.json').write_text(json.dumps(manifest,indent=2))
with (OUT/'bill_of_materials.csv').open('w',newline='',encoding='utf-8') as stream:
    writer=csv.DictWriter(stream,fieldnames=['name','category','material','printable','volume_mm3','notes'])
    writer.writeheader()
    for a in manifest:writer.writerow({k:a[k] for k in writer.fieldnames})
print(json.dumps(report,indent=2))
if collisions or not rt.isValid() or len(rt.Solids)!=len(parts):
    raise RuntimeError('Review CAD validation report before release')
