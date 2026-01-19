#pragma once

#include <vector>
#include <optional>
#include <limits>
#include <embree4/rtcore.h>

#include "aggregate/aggregate.hpp"
#include "shape/primitive.hpp"

namespace pbpt::aggregate {

template<typename T>
class EmbreeAggregate : public Aggregate<EmbreeAggregate<T>, T> {
    friend class Aggregate<EmbreeAggregate<T>, T>;

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
        bounds_o->lower_x = static_cast<float>(bbox.min().x());
        bounds_o->lower_y = static_cast<float>(bbox.min().y());
        bounds_o->lower_z = static_cast<float>(bbox.min().z());
        bounds_o->upper_x = static_cast<float>(bbox.max().x());
        bounds_o->upper_y = static_cast<float>(bbox.max().y());
        bounds_o->upper_z = static_cast<float>(bbox.max().z());
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

        math::Point<T, 3> o(ray.org_x, ray.org_y, ray.org_z);
        math::Vector<T, 3> d(ray.dir_x, ray.dir_y, ray.dir_z);
        geometry::Ray<T, 3> geo_ray(o, d, ray.tfar, ray.tnear);

        auto hit_record = primitive.intersect(geo_ray);

        if (hit_record) {
            // Update ray.tfar
            T t = hit_record->intersection.t;
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

        math::Point<T, 3> o(ray->org_x, ray->org_y, ray->org_z);
        math::Vector<T, 3> d(ray->dir_x, ray->dir_y, ray->dir_z);
        geometry::Ray<T, 3> geo_ray(o, d, ray->tfar, ray->tnear);

        if (primitive.is_intersected(geo_ray)) {
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
    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_impl(const geometry::Ray<T, 3>& ray) const {
        if (!m_scene) return std::nullopt;

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
            // primitive.intersect() to fail (t <= t_max). 
            // Since Embree has identified this primitive as the winner, we can safely use the 
            // original ray's t_max (or essentially infinity/large enough) to recover the exact interaction.
            
            // Optimization: we could just call primitive.intersect again
            // Or we could try to cache the result? But re-intersect is robust.
            return m_primitives[id].intersect(hit_ray);
        }
        
        return std::nullopt;
    }
    
    std::optional<T> is_intersected_impl(const geometry::Ray<T, 3>& ray) const {
        if (!m_scene) return std::nullopt;

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
        
        if (ray_e.tfar < 0.0f) { 
             return 0.0; 
        }
        return std::nullopt;
    }
};

}
