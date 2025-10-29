#pragma once

namespace pbpt::shape {

template<typename T, typename Derived>
class Shape {
public:

    constexpr Derived& as_derived() noexcept {
        return static_cast<Derived&>(*this);
    }

    constexpr const Derived& as_derived() const noexcept {
        return static_cast<const Derived&>(*this);
    }

    

};

};