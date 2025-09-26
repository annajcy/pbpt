#pragma once

namespace pbpt::core {

// TODO: shape
template<typename Derived>
class Shape {
public:

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }
    

};

class Sphere : public Shape<Sphere> {

};

};