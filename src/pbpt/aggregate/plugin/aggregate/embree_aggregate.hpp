#pragma once

#include <vector>
#include <optional>
#include <limits>
#include <embree4/rtcore.h>
#include <embree4/rtcore_common.h>

#include "pbpt/aggregate/aggregate.hpp"
#include "pbpt/shape/primitive.hpp"

namespace pbpt::aggregate {

template<typename T>
class EmbreeAggregate : public Aggregate<EmbreeAggregate<T>, T> {
    friend class Aggregate<EmbreeAggregate<T>, T>;

    // Thread-local pointer to the current high-precision ray being traced.
    // This allows the callbacks to access the original double-precision origin/direction/t_min
    // to avoid self-intersection artifacts caused by float quantization of the Embree ray.
    static inline thread_local const geometry::Ray<T, 3>* m_current_ray = nullptr;

private:
    std::vector<shape::Primitive<T>> m_primitives;
    RTCDevice m_device = nullptr;
    RTCScene m_scene = nullptr;

    static void boundsFunc(const RTCBoundsFunctionArguments* args) {
        const auto* primitives = static_cast<const shape::Primitive<T>*>(args->geometryUserPtr);
        const auto& primitive = primitives[args->primID];
        
        geometry::Bounds<T, 3> bbox;
        bool set = false;
        primitive.visit_shape([&](const auto& s) { 
             bbox = s.bounding_box(); 
             set = true;
        });

        if (!set) return;
        
        RTCBounds* bounds_o = args->bounds_o;
        // Widen bounds slightly to account for float conversion precision loss
        constexpr float eps = 1e-5f;
        bounds_o->lower_x = static_cast<float>(bbox.min().x()) - eps;
        bounds_o->lower_y = static_cast<float>(bbox.min().y()) - eps;
        bounds_o->lower_z = static_cast<float>(bbox.min().z()) - eps;
        bounds_o->upper_x = static_cast<float>(bbox.max().x()) + eps;
        bounds_o->upper_y = static_cast<float>(bbox.max().y()) + eps;
        bounds_o->upper_z = static_cast<float>(bbox.max().z()) + eps;
    }

    static void intersectFunc(const RTCIntersectFunctionNArguments* args) {
        if (args->N != 1) return; // Only support single ray for now
        int* valid = args->valid;
        if (!valid[0]) return;

        const auto* primitives = static_cast<const shape::Primitive<T>*>(args->geometryUserPtr);
        const auto& primitive = primitives[args->primID];
        
        RTCRayHit* rayhit = (RTCRayHit*)args->rayhit;
        RTCRay& ray = rayhit->ray;
        RTCHit& hit = rayhit->hit;

        // Use high-precision origin, direction and t_min from the original ray
        // combined with the current t_max (tfar) from Embree's traversal state.
        const auto* original_ray = m_current_ray;
        geometry::Ray<T, 3> geo_ray(
            original_ray->origin(), 
            original_ray->direction(), 
            static_cast<T>(ray.tfar), 
            original_ray->t_min()
        );

        auto hit_record = primitive.intersect_ray(geo_ray);

        if (hit_record) {
            // Update ray.tfar
            T t = hit_record->intersection.t;
            // Robustly update ray.tfar; ensure we don't accidentally exclude this hit in future checks
            // if we were to interact with mixed precision again.
            ray.tfar = static_cast<float>(t);

            // Update hit
            hit.geomID = args->geomID;
            hit.primID = args->primID; 
            
            auto n = hit_record->intersection.interaction.n();
            hit.Ng_x = static_cast<float>(n.x());
            hit.Ng_y = static_cast<float>(n.y());
            hit.Ng_z = static_cast<float>(n.z());
            
            auto uv = hit_record->intersection.interaction.uv();
            hit.u = static_cast<float>(uv.x());
            hit.v = static_cast<float>(uv.y());
        }
    }
    
    static void occludedFunc(const RTCOccludedFunctionNArguments* args) {
        if (args->N != 1) return;
        int* valid = args->valid;
        if (!valid[0]) return;

        const auto* primitives = static_cast<const shape::Primitive<T>*>(args->geometryUserPtr);
        const auto& primitive = primitives[args->primID];
        
        RTCRay* ray = (RTCRay*)args->ray;

        const auto* original_ray = m_current_ray;
        geometry::Ray<T, 3> geo_ray(
            original_ray->origin(), 
            original_ray->direction(), 
            static_cast<T>(ray->tfar), 
            original_ray->t_min()
        );

        if (primitive.is_intersected_ray(geo_ray)) {
            ray->tfar = -std::numeric_limits<float>::infinity(); 
        }
    }

public:
    EmbreeAggregate() {
        m_device = rtcNewDevice(nullptr);
        // Assuming success. In production check error.
    }
    
    EmbreeAggregate(std::vector<shape::Primitive<T>> primitives) 
        : m_primitives(std::move(primitives)) {
        
        m_device = rtcNewDevice(nullptr);
        m_scene = rtcNewScene(m_device);
        
        if (!m_primitives.empty()) {
            RTCGeometry geom = rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_USER);
            rtcSetGeometryUserPrimitiveCount(geom, m_primitives.size());
            rtcSetGeometryUserData(geom, (void*)m_primitives.data());
            
            rtcSetGeometryBoundsFunction(geom, boundsFunc, nullptr);
            rtcSetGeometryIntersectFunction(geom, intersectFunc);
            rtcSetGeometryOccludedFunction(geom, occludedFunc);
            
            rtcCommitGeometry(geom);
            rtcAttachGeometry(m_scene, geom);
            rtcReleaseGeometry(geom); // Scene holds reference
        }
        
        rtcCommitScene(m_scene);
    }
    
    ~EmbreeAggregate() {
        if (m_scene) rtcReleaseScene(m_scene);
        if (m_device) rtcReleaseDevice(m_device);
    }

    EmbreeAggregate(const EmbreeAggregate&) = delete;
    EmbreeAggregate& operator=(const EmbreeAggregate&) = delete;

    EmbreeAggregate(EmbreeAggregate&& other) noexcept 
        : m_primitives(std::move(other.m_primitives)), m_device(other.m_device), m_scene(other.m_scene) {
        other.m_device = nullptr;
        other.m_scene = nullptr;
    }
    
    EmbreeAggregate& operator=(EmbreeAggregate&& other) noexcept {
        if (this != &other) {
            if (m_scene) rtcReleaseScene(m_scene);
            if (m_device) rtcReleaseDevice(m_device);
            m_primitives = std::move(other.m_primitives);
            m_device = other.m_device;
            m_scene = other.m_scene;
            other.m_device = nullptr;
            other.m_scene = nullptr;
        }
        return *this;
    }

private:
    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_ray_impl(const geometry::Ray<T, 3>& ray) const {
        if (!m_scene) return std::nullopt;

        m_current_ray = &ray;

        RTCRayHit rayhit;
        rayhit.ray.org_x = static_cast<float>(ray.origin().x());
        rayhit.ray.org_y = static_cast<float>(ray.origin().y());
        rayhit.ray.org_z = static_cast<float>(ray.origin().z());
        rayhit.ray.dir_x = static_cast<float>(ray.direction().x());
        rayhit.ray.dir_y = static_cast<float>(ray.direction().y());
        rayhit.ray.dir_z = static_cast<float>(ray.direction().z());
        rayhit.ray.tnear = static_cast<float>(ray.t_min());
        rayhit.ray.tfar = static_cast<float>(ray.t_max());
        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID; // safety
        
        RTCIntersectArguments args;
        rtcInitIntersectArguments(&args);
        
        rtcIntersect1(m_scene, &rayhit, &args);

        m_current_ray = nullptr;
        
        if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            // Need to retrieve full interaction details.
            // Since we know the primitive index:
            unsigned int id = rayhit.hit.primID; // For USER Geom, primID is correctly set by us?
            // Wait, for User Geometry, we are responsible for setting hits.
            // In intersectFunc, we set: hit.primID = args->primID;
            // So yes, we get it back.
            
            if (id >= m_primitives.size()) return std::nullopt;
            
            // Re-intersect to get full record with T precision
            geometry::Ray<T, 3> hit_ray = ray;
            // Note: Do NOT use rayhit.ray.tfar (float) as the t_max for re-intersection.
            // Converting float tfar back to T (double) might result in a value slightly smaller 
            // than the actual intersection t due to precision loss, causing the check inside 
            // primitive.intersect_ray() to fail (t <= t_max). 
            // Since Embree has identified this primitive as the winner, we can safely use the 
            // original ray's t_max (or essentially infinity/large enough) to recover the exact interaction.
            
            // Optimization: we could just call primitive.intersect again
            // Or we could try to cache the result? But re-intersect is robust.
            return m_primitives[id].intersect_ray(hit_ray);
        }
        
        return std::nullopt;
    }

    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_ray_differential_impl(const geometry::RayDifferential<T, 3>& ray_diff) const {
        auto hit = intersect_ray_impl(ray_diff.main_ray());
        if (hit) {
            hit->intersection.differentials = hit->intersection.interaction.compute_differentials(ray_diff);
        }
        return hit;
    }
        
    std::optional<T> is_intersected_ray_impl(const geometry::Ray<T, 3>& ray) const {
        if (!m_scene) return std::nullopt;

        m_current_ray = &ray;

        RTCRay ray_e;
        ray_e.org_x = static_cast<float>(ray.origin().x());
        ray_e.org_y = static_cast<float>(ray.origin().y());
        ray_e.org_z = static_cast<float>(ray.origin().z());
        ray_e.dir_x = static_cast<float>(ray.direction().x());
        ray_e.dir_y = static_cast<float>(ray.direction().y());
        ray_e.dir_z = static_cast<float>(ray.direction().z());
        ray_e.tnear = static_cast<float>(ray.t_min());
        ray_e.tfar = static_cast<float>(ray.t_max());
        ray_e.mask = -1;
        ray_e.flags = 0;

        RTCOccludedArguments args;
        rtcInitOccludedArguments(&args);
        
        rtcOccluded1(m_scene, &ray_e, &args);

        m_current_ray = nullptr;
        
        if (ray_e.tfar < 0.0f) { 
             return 0.0; 
        }
        return std::nullopt;
    }
};

} // namespace pbpt::aggregate
