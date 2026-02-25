from pathlib import Path
import mitsuba as mi
import numpy as np

root = Path('/Users/jinceyang/Desktop/codebase/graphics/rtr2/external/pbpt')
scene = root / 'output/cbox/scene_export/cbox_checkerboard_texture.xml'
scene = root / 'asset/scene/cbox/cbox_checkerboard_texture.xml'
mi.set_variant('scalar_spectral')
resolver = mi.Thread.thread().file_resolver()
resolver.append(str(root))
resolver.append(str(scene.parent))

s = mi.load_file(str(scene))
img = mi.render(s, spp=2, seed=1)
mi.Bitmap(img).write(str(root / 'output/cbox/scene_export/cbox_render.exr'))
arr = np.array(mi.Bitmap(img).convert(mi.Bitmap.PixelFormat.RGB, mi.Struct.Type.Float32, srgb_gamma=False))
print(float(arr.max()), float(arr.mean()))