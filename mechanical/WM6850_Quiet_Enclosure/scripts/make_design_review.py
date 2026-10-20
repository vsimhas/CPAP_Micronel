"""Generate the six-page design review with reportlab (no CAD dependency)."""
from pathlib import Path
import json, math
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor, white
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.lib.styles import ParagraphStyle

ROOT=Path(__file__).resolve().parents[1]; OUT=ROOT/'output'
P=json.loads((ROOT/'parameters.json').read_text())
for name,path in [('Segoe','C:/Windows/Fonts/segoeui.ttf'),('SegoeB','C:/Windows/Fonts/segoeuib.ttf')]:
    pdfmetrics.registerFont(TTFont(name,path))
pdfmetrics.registerFontFamily('Segoe',normal='Segoe',bold='SegoeB',italic='Segoe',boldItalic='SegoeB')
INK=HexColor('#193a49');MUTED=HexColor('#526c78');TEAL=HexColor('#128892')
LIGHT=HexColor('#edf3f5');ORANGE=HexColor('#b95c28');LINE=HexColor('#cdd9df')
W,H=A4
c=canvas.Canvas(str(OUT/'Design_Review.pdf'),pagesize=A4)
c.setTitle('WM6850 Quiet Enclosure - Rev A - Bench Prototype')
c.setAuthor('Engineering concept prepared with Codex')
style=ParagraphStyle('Body',fontName='Segoe',fontSize=9.5,leading=14,textColor=INK)
small=ParagraphStyle('Small',parent=style,fontSize=8,leading=11)
tiny=ParagraphStyle('Tiny',parent=style,fontSize=7,leading=9)

def para(text,x,y,w,st=style):
    p=Paragraph(text,st);_,h=p.wrap(w,1000);p.drawOn(c,x,y-h);return y-h
def text(x,y,s,size=10,bold=False,col=INK):
    c.setFillColor(col);c.setFont('SegoeB' if bold else 'Segoe',size);c.drawString(x,y,s)
def page(n,title,subtitle):
    c.setFillColor(TEAL);c.rect(34,H-49,25,4,fill=1,stroke=0)
    text(34,H-78,title,22,True)
    para(subtitle,34,H-92,W-68,small)
    c.setStrokeColor(LINE);c.line(34,42,W-34,42)
    text(34,27,'WM6850  |  REV A  |  19 OCT 2026  |  BENCH PROTOTYPE',7,col=MUTED)
    c.setFont('Segoe',7);c.drawRightString(W-34,27,f'{n} / 6')
def label(title,body,x,y,w):
    text(x,y,title,11,True,TEAL);return para(body,x,y-10,w)
def table(rows,widths,x,y,font=8.2):
    st=ParagraphStyle('Table',parent=style,fontSize=font,leading=font+3)
    data=[[Paragraph(str(v),st) for v in row] for row in rows]
    t=Table(data,colWidths=widths,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),LIGHT),('VALIGN',(0,0),(-1,-1),'TOP'),
        ('LINEBELOW',(0,0),(-1,0),.7,TEAL),('LINEBELOW',(0,1),(-1,-1),.35,LINE),
        ('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),
        ('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7)]))
    _,h=t.wrap(0,0);t.drawOn(c,x,y-h);return y-h
def arrow(x1,y1,x2,y2,col=TEAL,width=1.7):
    c.setStrokeColor(col);c.setFillColor(col);c.setLineWidth(width);c.line(x1,y1,x2,y2)
    a=math.atan2(y2-y1,x2-x1);d=5
    p=c.beginPath();p.moveTo(x2,y2)
    p.lineTo(x2-d*math.cos(a-.5),y2-d*math.sin(a-.5));p.lineTo(x2-d*math.cos(a+.5),y2-d*math.sin(a+.5));p.close()
    c.drawPath(p,fill=1,stroke=0)
def hdim(x1,x2,y,label):
    c.setStrokeColor(MUTED);c.setLineWidth(.5);c.line(x1,y-5,x1,y+5);c.line(x2,y-5,x2,y+5);c.line(x1,y,x2,y)
    c.setFont('Segoe',8);c.setFillColor(INK);c.drawCentredString((x1+x2)/2,y+4,label)
def vdim(x,y1,y2,label):
    c.setStrokeColor(MUTED);c.setLineWidth(.5);c.line(x-5,y1,x+5,y1);c.line(x-5,y2,x+5,y2);c.line(x,y1,x,y2)
    c.saveState();c.translate(x-5,(y1+y2)/2);c.rotate(90);c.setFont('Segoe',8);c.setFillColor(INK);c.drawCentredString(0,0,label);c.restoreState()

# 1 - concept
page(1,'A quieter enclosure for WM6850','A serviceable 3D-printed concept for a CPAP blower. Acoustic, thermal and breathing-path performance remain unverified.')
c.drawImage(str(OUT/'cutaway.png'),24,310,width=W-48,height=(W-48)*1500/2200,mask='auto')
text(34,293,'220 x 160 mm body  /  118 mm overall height  /  4 printed parts',12,True)
label('1  Redirect intake sound','Two alternating baffles remove a direct sight line to the blower. A gasket seals both baffle tops to the lid.',34,258,160)
label('2  Isolate the blower','Four soft mounts support a separate carrier. A flexible hose and slack cable limit vibration transfer to the box.',216,258,160)
label('3  Dampen the lid','A metal cap over an external viscoelastic layer adds mass and damping. No exposed acoustic foam is specified.',398,258,163)
c.setFillColor(LIGHT);c.roundRect(34,105,W-68,76,6,fill=1,stroke=0)
para('<b>CAD handoff:</b> STEP assembly, individual STEP solids, printable STL files, FreeCAD project and regeneration source. Native SolidWorks files require conversion in SolidWorks; that application was not available for verification.',47,166,W-94)
para('<b>Before fabrication:</b> verify the actual blower mounting holes, port fit, screw depth and wire exits. This design is a bench prototype, not a validated patient-use assembly.',34,87,W-68,small)
c.showPage()

# 2 - overall dimensioned arrangement
page(2,'Overall arrangement','All dimensions in mm. Schematic orthographic arrangement derived from the CAD parameters; not a production tolerance drawing.')
x0,y0,s=110,400,1.6
def xy(x,y):return x0+x*s,y0+y*s
def rect(x,y,w,h,fill=LIGHT,stroke=INK):
    c.setFillColor(fill);c.setStrokeColor(stroke);c.setLineWidth(.65);c.rect(*xy(x,y),w*s,h*s,fill=1,stroke=1)
rect(0,0,220,160);rect(4,4,212,152,white)
rect(36,4,4,112,HexColor('#6b8998'));rect(76,44,4,112,HexColor('#6b8998'))
rect(104,43,100,90,HexColor('#d8edef'),TEAL)
for x,y in P['isolator_centres']:
    c.setFillColor(ORANGE);c.circle(*xy(x,y),6*s,fill=1,stroke=0)
c.setFillColor(HexColor('#d1d7db'));c.setStrokeColor(MUTED);c.circle(*xy(151,87),33.705*s,fill=1,stroke=1)
c.circle(*xy(151,87),7.1*s,fill=0,stroke=1)
rect(157.7,122,28,66,HexColor('#e7bda8'),ORANGE)
rect(-6,4,6,64,HexColor('#a5cfd2'),TEAL)
for x,y in P['lid_fasteners']:
    c.setStrokeColor(MUTED);c.circle(*xy(x,y),2.3*s,fill=0,stroke=1)
path=[(-10,34),(20,34),(20,136),(58,136),(58,24),(100,24),(122,45)]
for a,b in zip(path,path[1:]):arrow(*xy(*a),*xy(*b))
text(*xy(122,18),'BLOWER CHAMBER',7,True)
text(*xy(4,168),'TOP - lid and gasket removed',9,True)
hdim(*[xy(0,0)[0],xy(220,0)[0]],y0+160*s+54,'220')
vdim(x0-32,y0,y0+160*s,'160')
hdim(x0+36*s,x0+40*s,y0-16,'4')
hdim(x0+76*s,x0+80*s,y0-16,'4')
text(102,373,'Baffles: X = 36..40 and 76..80. Opposite turn gaps = 40.',8,col=MUTED)
text(102,359,'Arrows indicate intake route only; the blower shape is approximate.',8,col=MUTED)

# Front elevation, through the blower chamber.
x1,y1,k=102,132,1.75
def xp(x,z):return x1+x*k,y1+z*k
c.setFillColor(LIGHT);c.setStrokeColor(INK);c.rect(x1,y1,220*k,104*k,fill=1,stroke=1)
c.setFillColor(HexColor('#a5cfd2'));c.rect(*xp(0,104),220*k,1*k,fill=1,stroke=0)
c.setFillColor(HexColor('#6b8998'));c.rect(*xp(0,105),220*k,4*k,fill=1,stroke=0)
c.setFillColor(MUTED);c.rect(*xp(30,109),160*k,k,fill=1,stroke=0)
c.setFillColor(HexColor('#c5cdd2'));c.rect(*xp(30,110),160*k,2*k,fill=1,stroke=1)
c.setDash(3,2);c.setStrokeColor(MUTED);c.rect(*xp(117.295,28),67.41*k,54.9*k,fill=0,stroke=1)
c.rect(*xp(104,20),100*k,4*k,fill=0,stroke=1);c.circle(*xp(171.7,55),17*k,fill=0,stroke=1);c.setDash()
for x in [6,196]:c.setFillColor(ORANGE);c.rect(*xp(x,-6),18*k,6*k,fill=1,stroke=0)
vdim(x1-32,y1-6*k,y1+112*k,'118 overall')
vdim(x1+220*k+22,y1,y1+104*k,'104 base')
text(102,113,'FRONT - internal reference envelope shown dashed',8,True)
para('Datum: X=0 left wall, Y=0 cable side, Z=0 underside of base. Lid underside Z=105; blower reference top Z=82.9. External hose/filter/gland envelope is 226 x 190 x 118 mm. Leave additional service space.',34,89,W-68,small)
c.showPage()

# 3 - interface control
page(3,'Interfaces to verify before printing','The supplier drawing fixes the main envelope. Items marked PROVISIONAL must be checked against a physical blower or supplier CAD.')
rows=[['Interface','Model value / location','Status'],
['Blower envelope','Diameter 67.41; overall height 54.90; motor diameter 56','Datasheet p.5'],
['Mounting','3 x M3, 120 degrees apart','Datasheet p.5'],
['Carrier hole pattern','Diameter 50 PCD; angles 36, 156, 276 deg','PROVISIONAL'],
['Blower mount plane','Centre X151, Y87; mount plane Z30; boss extends to Z28','Design placement'],
['Outlet fitting','Barrel OD21.90; lip OD24; tip 44 from blower centre','Datasheet p.5'],
['Outlet placement','Axis X171.7, Z55; tip Y131; bore diameter 18','Offset/bore PROVISIONAL'],
['Wall passthrough','Diameter 34 at X171.7, Z55 in Y=160 wall','Design dimension'],
['Carrier isolators','X/Y: (113,52), (195,52), (113,124), (195,124); diameter 12 x 10','Generic purchased envelope'],
['Filter media','44 x 70 x 2; 36 x 62 window with two 3 mm bars','Media supplier TBD'],
['Cable opening','Diameter 16 at X190, Z30 in Y=0 wall; split gland bore 9','Check actual harness'],
['M3 insert pockets','Diameter 4.6 x 6 deep; 3.5 screw clearance','Match chosen insert']]
y=table(rows,[114,270,143],34,H-129,8.3)
label('Carrier fit check',
      'Print the carrier first. Match all three blower holes without force, check the central boss clearance and confirm screw engagement. The CAD does not establish the motor thread depth.',34,y-26,249)
label('Outlet and intake fit check',
      'Test a hose sample on the 24 mm lip. The top opening, volute contour and wire exits are simplified reference geometry; replace the blower envelope when a supplier STEP is available.',309,y-26,252)
c.setFillColor(LIGHT);c.roundRect(34,85,W-68,65,6,fill=1,stroke=0)
para('<b>Do not infer a patient connector from the hose size.</b> The outlet ends as a plain flexible tube. The existing CPAP interface, sensors and protective functions require their own specified fittings and system verification.',47,136,W-94,small)
c.showPage()

# 4 - BOM and fabrication
page(4,'Parts and fabrication','Print one of each of the four STL files. Gaskets, vibration mounts, filters and hardware are separate purchased or fabricated items.')
rows=[['Item','Qty','Specification / action'],
['Base with labyrinth','1','Print upright; 220 x 160 x 104. Local support at wall ports.'],
['Lid','1','Print flat; 220 x 160 x 4. Six M3 clearance holes.'],
['Blower carrier','1','Print flat; 100 x 90 x 4 plus three 6 mm spacers. Fit first.'],
['Filter retainer','1','Print in supplied STL orientation; 64 x 90 face.'],
['Rim/baffle + filter gaskets','1 each','Cut nonporous silicone. Rim: 1.5 free / 1 fitted; frame: 1 fitted.'],
['M3 bonded soft mounts','4','Nominal diameter 12 x 10. Select stiffness for actual load.'],
['Flexible outlet sleeve','1','Nominal 22 ID / 28 OD x 66; fit-check blower lip and clamps.'],
['Outlet grommet / split gland','1 each','Custom soft silicone: 34 hole / 28 bore; 16 hole / 9 bore.'],
['Intake filter','1','44 x 70 x 2 envelope; choose verified low-loss media and edge seal.'],
['External damping laminate','1 set','160 x 110 x 1 viscoelastic sheet + 160 x 110 x 2 aluminium.'],
['External feet','4','Diameter 18 x 6 nominal adhesive elastomer feet.'],
['M3 heat-set inserts','14','6 lid + 4 mount bases + 4 filter. Match pilot and depth.'],
['M3 screws, nuts, washers','See note','6 lid, 4 filter, 3 blower screws; 4 carrier nuts + washers.'],
['Hose clamps / strain relief','As needed','Select after fit checks; envelopes omitted from CAD.']]
y=table(rows,[145,50,332],34,H-127,8.2)
para('<b>Bench-print starting point:</b> PETG or ASA, 0.2 mm layers, 5-6 perimeters, 6 top/bottom layers, 40-60% infill. A brim needs a bed larger than the 220 mm part length. Printed volume is about 770 cm3 before slicer effects; the aluminium plate is about 95 g.',34,y-20,W-68,small)
para('<b>Hardware lengths:</b> M3 x 10 is only a lid starting point. Choose every screw/stud length from actual washer stack, insert and motor thread depths. The carrier-to-blower screw path is 10 mm before thread engagement. No clamp, insert or thread is modelled.',34,y-77,W-68,small)
c.showPage()

# 5 - assembly
page(5,'Assembly sequence','Use the exploded view for component order. Locations are fixed in the neutral assembly; hardware is deliberately omitted.')
c.drawImage(str(OUT/'exploded.png'),78,358,width=W-156,height=(W-156)*1500/2200,mask='auto')
steps=[
('01  Check the interfaces','Fit the carrier and hose sample to the actual blower. Update the model before printing the box.'),
('02  Install the floating carrier','Fit base inserts, four isolators, carrier and blower. No rigid attachment should bypass the isolators.'),
('03  Connect outlet and harness','Fit hose/grommet and split cable gland. Add retention, strain relief and an internal slack cable loop.'),
('04  Seal the intake','Fit specified filter media, seal its edge and screw down the gasketed filter retainer.'),
('05  Close and damp the enclosure','Fit the rim/baffle gasket and lid, tighten evenly, attach external damping laminate and feet.'),
('06  Verify on the bench','Check clearance, leakage, restriction, noise, temperatures and transient pressure response before release.')]
y=340
for title,body in steps:
    text(34,y,title,10,True,TEAL);para(body,34,y-9,W-68,small);y-=46
c.showPage()

# 6 - validation and citations
page(6,'Validation status and next measurements','The files are ready for CAD review and fit prototyping. No acoustic attenuation, CFD result or temperature qualification is claimed.')
y=label('Passed: geometry and exchange',
    '22 valid single solids; no overlaps above 0.01 mm3; STEP re-import contains 22 valid solids. Four printed STLs are closed meshes. SolidWorks import and the optional native-conversion macro remain untested.',34,H-139,W-68)
y=label('Measure: noise at matched delivered performance',
    'Compare bare versus enclosed at the same flow and delivered pressure, using the same hose and load. Record LAeq and spectra at 1 m at inlet, front and outlet sides. A proposed development target is at least 5 dB(A) insertion loss at an agreed operating point; this is not a prediction. The datasheet 45 dB(A) applies to its specified 1 kPa / 4 mm-orifice setup.',34,y-27,W-68)
y=label('Measure: restriction and pressure response',
    'Grille open area: 2,016 mm2; nominal minimum labyrinth channel: 3,200 mm2. At 300 L/min, grille velocity is 2.48 m/s. Assuming K=10 gives about 37 Pa of geometric loss, excluding filter and blower-entry losses; K is only a sensitivity assumption. Test actual losses with new and loaded filters. Proposed characterization: 4/10/20 cmH2O at 30/60/120 L/min, higher flows only where available.',34,y-27,W-68)
y=label('Measure: temperatures, seals and materials',
    'Log motor NTC, housing, outlet air and plastic-boss temperatures to steady state. An all-heat-to-air sensitivity gives about 17 degC rise at 120 L/min and 40.8 W; it is not a thermal prediction. Check leaks, hose retention, filter bypass, loose particles and cleanability. The datasheet prohibits complete blockage beyond 3 minutes; do not use prolonged occlusion tests.',34,y-27,W-68)
y=label('Required before patient use',
    'Qualify the complete breathing-gas pathway and printed surfaces, including particulate and volatile emissions, cleaning and system performance. No foam is exposed, but that alone does not qualify the materials. Confirm application-specific thermal and pressure limits before testing.',34,y-27,W-68)
text(34,y-27,'Sources and exchange guidance',10,True,TEAL)
sources=[
'User-supplied WM6850-24V specification, issued 2024-06-12: p.3 ratings and noise conditions; p.5 outline drawing. Original location is recorded in README.md.',
'<link href="https://help.solidworks.com/2024/english/SolidWorks/sldworks/t_reading_step_iges_acis_sw.htm" color="#128892">SolidWorks: importing STEP, IGES and ACIS files</link>. Import as an assembly, verify mm units and save referenced native parts.',
'<link href="https://www.cdc.gov/niosh/noise/prevent/engineering.html" color="#128892">NIOSH: engineering noise controls</link>. Supports enclosure and isolation principles, not design-specific attenuation.',
'<link href="https://www.accessdata.fda.gov/scripts/cdrh/cfdocs/cfStandards/detail.cfm?standard__identification_no=45211" color="#128892">FDA: recognized ISO 18562-3:2024</link>. Evaluation of volatile emissions in breathing-gas pathways.'
]
y-=39
for source in sources:y=para(source,34,y,W-68,tiny)-6
if y<52:raise ValueError(f'Page 6 footer collision: {y}')
c.save()
print(OUT/'Design_Review.pdf')
