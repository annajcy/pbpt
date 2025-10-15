#include "src/camera/spherical_camera.hpp"
#include "src/camera/camera.hpp"
#include <iostream>
#include <iomanip>

using namespace pbpt::camera;
using namespace pbpt;

int main() {
    math::Vector<int, 2> resolution(360, 180);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Testing SphericalCamera mapping:\n";
    std::cout << "Resolution: " << resolution.x() << "x" << resolution.y() << "\n\n";
    
    struct TestPoint {
        float film_x, film_y;
        const char* name;
    };
    
    std::vector<TestPoint> tests = {
        {0.0f, 0.0f, "Top-Left (0,0)"},
        {180.0f, 0.0f, "Middle-Left (180,0)"},
        {360.0f, 0.0f, "Bottom-Left (360,0)"},
        {0.0f, 90.0f, "Top-Center (0,90)"},
        {180.0f, 90.0f, "Middle-Center (180,90)"},
        {360.0f, 90.0f, "Bottom-Center (360,90)"},
        {0.0f, 180.0f, "Top-Right (0,180)"},
        {180.0f, 180.0f, "Middle-Right (180,180)"},
        {90.0f, 0.0f, "Quarter-Left (90,0)"},
        {90.0f, 45.0f, "(90,45)"},
        {90.0f, 90.0f, "(90,90)"},
        {90.0f, 135.0f, "(90,135)"},
        {270.0f, 90.0f, "(270,90)"},
    };
    
    for (const auto& test : tests) {
        CameraSample<float> sample;
        sample.p_film = math::Point<float, 2>(test.film_x, test.film_y);
        
        float u = test.film_x / 360.0f;
        float v = test.film_y / 180.0f;
        float theta = 3.14159265f * u;
        float phi = 2.0f * 3.14159265f * v;
        
        auto ray = camera.generate_ray(sample);
        
        std::cout << test.name << ":\n";
        std::cout << "  Film: (" << test.film_x << ", " << test.film_y << ")\n";
        std::cout << "  UV: (" << u << ", " << v << ")\n";
        std::cout << "  Theta: " << theta << " (" << (theta * 180.0f / 3.14159265f) << "°)\n";
        std::cout << "  Phi: " << phi << " (" << (phi * 180.0f / 3.14159265f) << "°)\n";
        std::cout << "  Direction: (" << ray.direction().x() << ", " 
                  << ray.direction().y() << ", " << ray.direction().z() << ")\n\n";
    }
    
    return 0;
}
