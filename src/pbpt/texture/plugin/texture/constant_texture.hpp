#pragma once

#include "pbpt/texture/texture.hpp"

namespace pbpt::texture {

template <typename T, typename ValueType>
class ConstantTexture : public Texture<ConstantTexture<T, ValueType>, T, ValueType> {
    // friend class Texture<ConstantTexture<T, ValueType>, T, ValueType>;
private:
    ValueType m_value{};

public:
    ConstantTexture() = default;
    explicit ConstantTexture(const ValueType& value) : m_value(value) {}

    ValueType eval_impl(const TextureEvalContext<T>&) const { return m_value; }
};

}  // namespace pbpt::texture
