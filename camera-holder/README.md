#

3D blender tutorial: https://www.youtube.com/watch?v=rN-HMVTB7nk
# Changes to original

## V2: Change to metric system

Original dimensions are in inches.
1. Apply scale 0.0254 to object
2. Set Unit System to metric
3. Set Unit Scale to 0.0001
4. Set Lenth to millimters
5. In Viewport overlays change scale to 0.01 (changes each grid cell to a centimeter)
6. Set origin to center of front of object. This is to easily move things around center.
   - Or move object to 0 on X and Y axis.

## V3: Remove some edges++ that causes issues.

- Seems to be some loose edges that can be removed.
- Do the steps bellow in 'Prepare for Pruca Slicer'
  - Check that not warnings.

## V4: Changes to LED holder

- Change width of camera holder to 10mm, this is sufficient for breadboard.
- Move holder generally down about 10mm, this is useful to give the
  wires a bit more space.
- Cut first poles of holder to avoid blocking LEDs. About 5mm is
  sufficient height. Also change to triangle
- Move down a bit

## V5: Remove old holes for screws

What seems to work:
1. In X-ray view, mark all faces in circle.
2. Press x, then "Collapse edges and faces"
3. Mark remaining edges and press x, and then "Dissolve edges"

- Make sure doesn't remove face/surface
- Make sure still is valid topolgoy after...

## V6: Change and add holes for screws


7. We want to have four 2mm holes for screws, 34 mm apart.
   - Create cylinder object from object mode. Rotate X and Y axis 90 degrees.
   - Create three copies
   - Edges need to be moved down.


## V7: Change and add holes for screws

- Add boolean operator to remove holes from object

## V8: Refine using Bevel tool

- Deleted

## Prepare for Prusa Slicer

- https://gemini.google.com/app/1468660cf8658bdb

To bring a model from Blender into PrusaSlicer, export your model in a supported format like 3MF or STL, then load and slice it in PrusaSlicer.

1. Prepare Your Model in Blender
  - Check Watertightness: Ensure your mesh is "manifold" with no holes or self-intersecting faces. Select your object, enter Edit Mode, go to Select > Select All by Trait > Non Manifold, and fix any highlighted vertices.
  - Apply Modifiers & Scale: If you used modifiers like Subdivision Surface or scaled the object in Object Mode, press Ctrl + A and select All Transforms to apply scale/rotation.
  - Orient the Normals: Select all faces in Edit Mode, press Shift + N to ensure normals face outward.

2. Export from Blender
  - Go to File > Export > 3MF (.3mf) (recommended for accurate scale and multi-part data) or STL (.stl).
  - In the export settings sidebar on the right:
  - Check Selection Only if you only want the highlighted object.
  - Adjust scale if needed (Blender’s default unit is 1 meter, so check your export scale if the object imports tiny).

3. Import into PrusaSlicer
  - Open PrusaSlicer and drag-and-drop your .3mf or .stl file onto the virtual print bed.
  - Use the left toolbar to Place on Face (F), Scale (S), or Rotate (R) to orient the flat base against the build plate.
  - Click Slice now in the bottom right corner, review the toolpath layer-by-layer, and click Export G-code to save it to your SD card or USB drive.

## Prusa Slicer

Export G-Code.

Tip from Alex: Set First layer expansion to 5mm

## Prusa printer

### Preheat and load filament

- Turn printer on, button on the back of the machine.
- Select Preheat on printer (e.g. dragonfly)
- Find 1.75mm PLA filament, choose 'Load filament'

### Upload gcode

Put: http://dragonfly.local/ in browser. Log in with:
- Username: pi
- Password: 3712

Actually you may also add this to 'Physical printer' tab on Raspberry Pi.