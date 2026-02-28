from pathlib import Path
import mitsuba as mi
import numpy as np
import drjit as dr

mi.set_variant('llvm_ad_rgb')

class SimplePathIntegrator(mi.SamplingIntegrator):
    def __init__(self, props=mi.Properties()):
        super().__init__(props)
        self.max_depth = props.get("max_depth", -1)
        self.rr_depth = props.get("rr_depth", 5)

    def sample(self, scene, sampler, ray_, medium, active):
        ray = mi.Ray3f(ray_)
        active = mi.Bool(active)
        throughput = mi.Spectrum(1.0)
        result = mi.Spectrum(0.0)
        depth = mi.UInt32(0)
        eta = mi.Float(1.0)

        valid_ray = scene.ray_intersect(ray_, active).is_valid()
        max_depth = mi.UInt32(self.max_depth if self.max_depth >= 0 else 100)
        rr_depth = mi.UInt32(self.rr_depth)

        @dr.syntax
        def run_loop(
            scene,
            sampler,
            ray,
            throughput,
            result,
            eta,
            depth,
            active,
            max_depth,
            rr_depth,
        ):
            while active:
                si = scene.ray_intersect(ray, active)
                escaped = active & ~si.is_valid()
                env = scene.environment()
                if env is not None:
                    result += dr.select(
                        escaped, throughput * env.eval(si, escaped), 0.0
                    )
                active &= si.is_valid()

                emitter = si.emitter(scene, active)
                hit_emitter = active & (emitter != None)
                result += dr.select(
                    hit_emitter, throughput * emitter.eval(si, hit_emitter), 0.0
                )

                bsdf = si.bsdf(ray)
                active &= bsdf != None

                ctx = mi.BSDFContext()
                bs, bsdf_val = bsdf.sample(
                    ctx,
                    si,
                    sampler.next_1d(active),
                    sampler.next_2d(active),
                    active,
                )
                throughput[active] *= bsdf_val
                active &= dr.max(throughput) != 0.0

                ray[active] = si.spawn_ray(si.to_world(bs.wo))
                eta[active] *= bs.eta
                depth += 1
                rr_active = active & (depth >= rr_depth)
                q = dr.minimum(dr.max(throughput) * dr.square(eta), 0.95)
                rr_continue = sampler.next_1d(rr_active) < q
                throughput[rr_active] *= dr.rcp(dr.detach(q))
                active &= ~rr_active | rr_continue

                if max_depth > 0:
                    active &= depth < max_depth

            return result

        result = run_loop(
            scene,
            sampler,
            ray,
            throughput,
            result,
            eta,
            depth,
            active,
            max_depth,
            rr_depth,
        )
        return result, valid_ray, []

mi.register_integrator("simple_path", lambda props: SimplePathIntegrator(props))

root = Path('/Users/jinceyang/Desktop/codebase/graphics/rtr2/external/pbpt')
scene_path = root / 'output/cbox/scene_export/cbox_checkerboard_texture.xml'
scene_path = root / 'asset/scene/cbox/cbox_microfacet_diele_iso.xml'

scene = mi.load_file(str(scene_path))
image = mi.render(scene, spp=256, seed=1)
mi.util.write_bitmap('cbox_render.exr', image)