# WM6850 acoustic enclosure - Rev A

Created 19 October 2026 for a **3D-printed CPAP blower bench prototype**. This package contains actual CAD solids, four printable parts, assembly geometry, drawings and the rebuild source. Noise reduction has not been measured. Do not use this prototype for patient therapy.

## Open the project

- `cad/WM6850_Quiet_Enclosure.step`: 22-component neutral CAD assembly for SolidWorks.
- `cad/WM6850_Quiet_Enclosure.FCStd`: editable FreeCAD project with named solids and a dimension register.
- `cad/step_parts/`: individual solid parts in assembly coordinates, in millimetres.
- `cad/stl_print/`: four closed meshes repositioned for printing, in millimetres. Print one of each.
- `output/Design_Review.pdf`: dimensioned arrangement, assembly instructions, BOM and verification plan.
- `output/cutaway.png`, `assembled.png`, `exploded.png`: views rendered from the CAD geometry.
- `parameters.json` and `scripts/build_enclosure.py`: regeneration source. The FreeCAD solids do not have a sketch/feature history; use the source to revise geometry. The dimension register is informational, not a live driver. Some component positions and purchased-part envelopes are explicitly fixed in the source and must be revised along with changed parameters.

**Native SolidWorks files were not generated:** SolidWorks is not installed on this workstation. The STEP assembly was re-imported and checked with FreeCAD/Open Cascade, not SolidWorks. Never rename a STEP file to `.SLDASM` or `.SLDPRT`.

### SolidWorks workflow

1. Open the assembly STEP with File > Open. Keep assembly structure; do not choose the option to import the whole assembly as a single multibody part. Check import units are mm.
2. Run Import Diagnostics and measure the base: **220 x 160 x 104 mm**. Confirm **22 components**. The lid lies at Z=105..109 mm; the feet at Z=-6..0 mm.
3. Save As a SolidWorks assembly. Where required by your version, dissolve/break the imported 3D Interconnect link and save referenced parts into the same new folder.
4. If assembly import flattens the structure, import the files in `step_parts` individually, save as parts, and insert at the shared origin with zero rotation. They already contain assembly coordinates.
5. Optional `solidworks/BuildNativeAssembly.bas` is VBA source for step 4. Import it into a new SolidWorks VBA macro and run `main`. It creates a timestamped output folder so existing parts cannot be overwritten. This helper is **not tested in SolidWorks**; manual import remains the verified format-based handoff.

Imported solids support direct editing or added features in SolidWorks; they do not reconstruct a native parametric feature history. A production SolidWorks drawing and native rebuild need SolidWorks and a verified supplier blower model.

## Layout and acoustic intent

- Body: **220 L x 160 W x 104 H mm**, 4 mm walls/floor, open top with screw-down lid. Lid plus compressed gasket: 5 mm. External damping layer and aluminium cap: 3 mm. Feet: 6 mm. Full rigid assembly envelope: **226 x 190 x 118 mm** including filter retainer, cable-gland lip and outlet hose. Leave hose bend and filter service space beyond this envelope.
- Two integral baffles at X=36..40 and X=76..80 mm end at opposite sides with 40 mm gaps. Air enters the left wall, turns around each baffle, and enters the blower chamber. The gasket seals baffle tops to the lid to avoid a straight acoustic shortcut. Smooth rigid baffles reflect sound; attenuation can be frequency dependent and is not predicted here.
- Four elastomer mounts separate the carrier from the base. Only the carrier is screwed to the blower. Use short, correct-length blower screws after checking the actual thread engagement depth. Leave cable slack and avoid rigid bridges between blower and housing.
- A 22 mm ID flexible sleeve connects the blower outlet through a soft wall grommet. The sleeve's external end is plain hose, **not a dimensioned ISO conical breathing connector**. Adapt it to the existing system with a separately specified fitting.
- A 160 x 110 x 2 mm aluminium plate over a 1 mm viscoelastic sheet damps the lid from outside the air chamber. The design contains no exposed porous acoustic foam. Material safety still requires evaluation of every gas-contact surface, seal and adhesive system.
- The motor controller, mains supply, humidifier, pressure/flow sensors and their plumbing are outside this enclosure; no board dimensions were supplied.

## Datasheet basis versus assumptions

Primary source: user-supplied `WM6850-24V-Brushless Blower Spcification.pdf`, issued 2024-06-12, pages 3 and 5. The source file remains at its original location; a rendered outline is in `reference/`.

| Item | Basis |
| --- | --- |
| 24 V, 1.7 A, 40.8 W rated; 150 g | Datasheet page 3 |
| 300 L/min at zero pressure; 5.2 kPa at zero flow | Separate endpoint ratings, not simultaneous performance |
| 45 dB(A) | Datasheet at 1 m from inlet, 1 kPa, outlet hose and 4 mm orifice, specified acoustic setup; not a universal baseline |
| Diameter 67.41; overall height 54.90; motor diameter 56 mm | Drawing page 5 |
| Outlet barrel 21.90; lip diameter 24 mm; tip 44 mm from centre | Drawing page 5 |
| 3 x M3, 120 degree spacing | Drawing page 5 |
| Diameter 50 mounting PCD; angular start 36 degrees | **Provisional interpretation** of drawing; check against real blower |
| Outlet X offset 20.7; bore 18 mm | **Provisional**; bore inferred from R9 side-view detail |
| Smooth volute approximation and 14.2 mm top opening | **Reference-only**, not a supplier CAD model; intake geometry not fully resolved |
| Insert pilot diameter 4.6 x 6 mm depth | **Provisional**; replace with the selected insert supplier's dimensions |
| Isolators, filter, grommets, hose and gland | **Generic envelopes**, no purchased SKU has been selected |

The motor thread depth, actual intake opening, connector envelope, cable exit direction and operating-point noise spectrum need confirmation. Fit-check the carrier and a short hose sample before committing to the complete print.

## Fabrication and assembly

1. Print `06_Isolated_blower_carrier` first. Check the mounting pitch, shaft/boss clearance, angular orientation and screw engagement on the physical blower. Check 22 ID hose fit over the 24 mm outlet lip. Adjust source and regenerate if necessary.
2. Print base upright, lid flat, carrier flat and filter frame in the supplied STL orientations. A build area larger than 220 x 160 mm is needed for the base once a brim is included. Bench starting settings: 0.2 mm layers, 5-6 perimeters, 6 top/bottom layers and 40-60% infill. Use local supports at horizontal wall ports and remove them completely. No strength, airtightness, temperature or breathing-path qualification follows from these settings.
3. Fit 14 M3 heat-set inserts to the measured pilot size: 6 for lid, 4 for isolator bases, 4 for filter frame. Base isolator pockets are blind and must retain an airtight floor. Selected insert length must fit the pocket.
4. Install four nominal diameter 12 x 10 mm bonded M3 elastomer mounts; keep the lower stud short enough for the 6 mm boss. Fasten carrier to top studs with four M3 nuts/washers. Mounts are illustrative envelopes: choose stiffness for the measured supported mass, ideally a mounted natural frequency below 40 Hz, and check rocking/transient loads. Geometry alone does not specify dynamic stiffness.
5. Fasten blower to carrier with three M3 screws/washers from below. Screw length must equal the 10 mm carrier-plus-spacer path plus the **verified** thread engagement. Do not bottom the screws in the blower.
6. Fit outlet sleeve, appropriate low-profile clamps and custom soft grommet. Keep at least approximately 23 mm of free sleeve between the blower lip at Y=131 and grommet flange at Y=154; verify dynamic movement and no kinking. The model omits clamps and hardware; measure their complete envelopes.
7. Route the actual harness through the split 9 mm bore gland, fit external strain relief and leave a flexible slack loop internally. Check preterminated connector passage through the 16 mm panel opening; enlarge and redesign the gland if required.
8. Install a specified 44 x 70 x 2 mm intake filter and seal its perimeter. Fit the filter frame with four M3 screws and gasket. Media permeability, particulate performance, retention and edge sealing are not defined by its CAD envelope.
9. Cut the combined rim/baffle gasket from nonporous silicone sheet; use 1.5 mm nominal free thickness compressed to the 1 mm model thickness, subject to material data. Fit six M3 lid screws with washers; approximately M3 x 10 is a starting size, but verify insert depth and washer thickness. Tighten evenly and leak-test.
10. Bond the external damping sheet and aluminium cap to the lid. Install four feet. Place temperature probes and complete bench tests before any application beyond a prototype.

Approximate fully dense printed CAD volume is 770 cm3; mass depends on polymer and slicer settings. The aluminium cap is approximately 95 g. Do not use the reference blower's solid CAD volume to estimate its mass; use the datasheet's 150 g.

## Validation and limitations

Geometry checks in `output/geometry_validation.json` confirm 22 valid single solids, no component overlaps above 0.01 mm3 and a valid STEP re-import with 22 solids. Four printable STLs were checked as closed meshes. Touching faces represent intended joints. Threads, washers, clamps and inserts are not modelled. These checks do not validate fit, fluid passage, print tolerances, acoustics, heat or clinical performance.

The intake grille has 2,016 mm2 geometric open area; the narrowest nominal labyrinth channel is 3,200 mm2. At 300 L/min the grille velocity is approximately 2.48 m/s; dynamic pressure is approximately 3.69 Pa at density 1.2 kg/m3. With an illustrative lumped loss coefficient K=10 referenced to that area, geometric-path loss would be about 37 Pa (0.38 cmH2O), **excluding filter loss and blower entry loss**. At 120 L/min this same assumption gives 5.9 Pa. K is not measured or calculated by CFD, so these are sensitivity estimates, not verified performance. Baffles and inlet area must be tested with the selected filter.

Airflow thermal sensitivity: if all 40.8 W entered the air, delta T = P/(rho*Q*cp) gives approximately 34, 17 and 6.8 degC at 60, 120 and 300 L/min (cp=1005 J/kg K). Real power and heat paths depend on operating point. This is not a temperature prediction; it shows why the datasheet's open-installation 12 degC surface rise cannot qualify an enclosed design.

Bench plan:

- Compare bare and enclosed blower at **matched flow and delivered pressure**, not merely matched RPM. Use the same hose, load, microphone geometry and background conditions. Record LAeq and spectra at 1 m at front, inlet and outlet sides. Initial development goal: at least 5 dB(A) insertion loss at an agreed representative operating point, with no new prominent tones. This is a proposed goal, not an achieved result.
- Sweep nominal 4, 10 and 20 cmH2O delivery targets with 30, 60 and 120 L/min loads; explore 200 L/min where the blower can supply it and 300 L/min only near free flow. These are proposed characterization points; the user has not yet specified an operating point. Measure intake restriction with new and loaded filters, electrical power and transient pressure response. Suggested preliminary allowance: under 50 Pa added intake loss at 120 L/min, including filter; confirm against system requirements.
- Log blower NTC, housing, outlet air and insert-boss temperatures until stable at low, high and restricted-flow loads. Limits must come from component, polymer and respiratory-system requirements. The datasheet ambient limit is -10..40 degC. Do not deliberately hold a fully blocked inlet/outlet; the datasheet prohibits blockage beyond 3 minutes.
- Check leak rate, loose-particle release, cleaning access, filter retention, hose pull-out, strain relief, movement clearance and mounting durability. Keep the existing pressure-protection architecture functional.
- Breathing-path materials and finished surfaces need appropriate qualification (including particles and volatile emissions under ISO 18562 as applicable), cleaning validation and system verification before patient use. A supplier's blower biocompatibility claim does not qualify this printed enclosure.

## Rebuild on this workstation

```powershell
& 'C:\Program Files\FreeCAD 1.1\bin\python.exe' '.\scripts\build_enclosure.py'
& 'C:\Program Files\FreeCAD 1.1\bin\python.exe' '.\scripts\render_views.py'
```

Run from this project folder. PDF generation uses `scripts/make_design_review.py` with the bundled Codex Python (reportlab). Changing dimensions requires reviewing placement constraints, seal geometry, hardware and all validation outputs; this is a prototype generator, not a general enclosure configurator.

## Supporting references

- [SolidWorks STEP import](https://help.solidworks.com/2024/english/SolidWorks/sldworks/t_reading_step_iges_acis_sw.htm) - neutral CAD handoff.
- [NIOSH engineering controls](https://www.cdc.gov/niosh/noise/prevent/engineering.html) - enclosure and vibration-isolation principles; does not establish attenuation for this design.
- [FDA foam testing summary](https://www.fda.gov/medical-devices/recalled-philips-ventilators-bipap-machines-and-cpap-machines/foam-testing-summary-recalled-philips-ventilators-bipap-machines-and-cpap-machines) - breathing-path foam degradation and emission concerns.
- [FDA recognized ISO 18562-3:2024](https://www.accessdata.fda.gov/scripts/cdrh/cfdocs/cfStandards/detail.cfm?standard__identification_no=45211) - volatile-substance evaluation of respiratory gas pathways.
