# RSPSpectrumTexture Technical Documentation

## Overview
`RSPSpectrumTexture` is a texture implementation designed to shift the computational cost of RGB-to-Spectrum conversion from runtime (during rendering) to load time (during scene initialization).

Instead of storing RGB values and performing a lookup for every ray intersection during rendering, this texture stores pre-computed RGB Sigmoid Polynomial (RSP) coefficients in its texels.

## Data Structure
The texture internally uses a `MipMap<T, math::Vector<T, 3>>`.
The three components of the vector store the `c0`, `c1`, `c2` coefficients of the `RGBSigmoidPolynomialNormalized`.

This structure maintains compatibility with existing MipMap logic, including trilinear filtering which is applied directly to the coefficients. Linearity of the polynomial coefficients allows for valid interpolation.

## Double Conversion Semantics
The class supports bidirectional conversion:
1.  **Bitamp to RSP (Load)**: Uses `radiometry::lookup_srgb_to_rsp` (based on PBRT's optimization) to find the best-fit sigmoid polynomial for a given linear RGB triplet.
2.  **RSP to Bitmap (Reconstruction)**: Re-evaluates the polynomial to a spectrum, computes XYZ under CIE D65 illuminant, and converts XYZ back to linear RGB using sRGB primaries. This is an approximate reconstruction.

## Error Evaluation
We evaluate the quality of the RSP representation using the roundtrip error (RGB -> RSP -> RGB).

### Metrics
*   **RMSE**: Root Mean Square Error of the RGB channels.
*   **DeltaE76**: Euclidean distance in CIE L*a*b* color space, providing a perceptual error metric.

### Observed Baseline
On a test gradient image:
*   RGB RMSE: ~0.028 (Linear space [0,1])
*   Avg DeltaE76: ~6.9
*   Max DeltaE76: ~33.3

These errors are due to the mathematical limits of fitting arbitrary spectra with a 3-coefficient sigmoid polynomial and the interpolation within the 64x64x64 lookup table. While perceptually lossy for some colors (DeltaE > 2.0), the spectral properties are generally preserved well enough for physically based rendering where spectral interactions are the primary goal.

## Performance
By preloading RSP coefficients:
*   We eliminate `lookup_srgb_to_rsp` (and associated 5D table lookups/interpolation) from `LambertianMaterial::compute_bsdf`.
*   We eliminate `create_srgb_albedo_spectrum` overhead per shading point.
*   Memory usage is identical to high-precision (float) RGB textures (3 floats per texel).

## Integration
The loader automatically uses `RSPSpectrumTexture` when `<texture type="bitmap">` is encountered. To fallback to legacy behavior, code modification in `scene_loader.hpp` is currently required.
