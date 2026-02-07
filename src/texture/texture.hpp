#pragma once

namespace pbpt::texture {

template<typename T>
struct TextureEvalContext {
    
};

template<typename Derived, typename T, typename ValueType>
class Texture {
public:
    Derived& derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& derived() const {
        return static_cast<const Derived&>(*this);
    }

    ValueType eval(const TextureEvalContext<T>& ctx) const {
        return derived().eval(ctx);
    }
};

template<typename T, typename ValueType>
class ConstantTexture : public Texture<ConstantTexture<T, ValueType>, T, ValueType> {
    friend class Texture<ConstantTexture<T, ValueType>, T, ValueType>;

};

}