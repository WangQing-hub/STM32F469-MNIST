/**
  ******************************************************************************
  * This file is part of the TouchGFX 4.16.0 distribution.
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/**
 * @file touchgfx/widgets/ScalableImage.hpp
 *
 * Declares the touchgfx::ScalableImage class.
 */
#ifndef SCALABLEIMAGE_HPP
#define SCALABLEIMAGE_HPP

#include <touchgfx/Bitmap.hpp>
#include <touchgfx/widgets/Image.hpp>

namespace touchgfx
{
/**
 * Widget for representing a scaled version of a bitmap. Simply change the width/height of the
 * widget to resize the image. The quality of the scaled image depends of the rendering
 * algorithm used. The rendering algorithm can be changed dynamically. Please note that
 * scaling images is done at runtime and may require a lot of calculations.
 *
 * @note Note that this widget does not support 1 bit per pixel color depth.
 */
class ScalableImage : public Image
{
public:
    /**
     * Rendering algorithm to use when scaling the bitmap. Nearest neighbor simply finds the
     * closest pixel in the source bitmap. Bi-linear interpolation averages 4 pixels to find
     * a much better pixel representation.
     */
    enum ScalingAlgorithm
    {
        NEAREST_NEIGHBOR,      ///< Fast but not a very good image quality. Good for fast animations.
        BILINEAR_INTERPOLATION ///< Slower but better image quality. Good for static representation of a scaled image.
    };

    /**
     * Constructs a new ScalableImage with a default alpha value of 255 (solid) and a
     * default Bitmap (undefined) if none is specified. If a Bitmap is passed to the
     * constructor, the width and height of this widget is set to those of the bitmap.
     *
     * @param  bitmap (Optional) The bitmap to display.
     *
     * @see setBitmap
     */
    ScalableImage(const Bitmap& bitmap = Bitmap());

    /**
     * Sets the algorithm to be used. In short, there is currently a value for fast (nearest
     * neighbor) and a value for slow (bi-linear interpolation).
     *
     * @param  algorithm The algorithm to use when rendering.
     *
     * @see ScalingAlgorithm
     */
    virtual void setScalingAlgorithm(ScalingAlgorithm algorithm);

    /**
     * Gets the algorithm used when rendering.
     *
     * @return The algorithm used when rendering.
     *
     * @see ScalingAlgorithm
     */
    virtual ScalingAlgorithm getScalingAlgorithm();

    virtual void draw(const Rect& invalidatedArea) const;

    virtual Rect getSolidRect() const;

protected:
    ScalingAlgorithm currentScalingAlgorithm; ///< The current scaling algorithm.

private:
    /// @cond
    /**
     * Draw a triangle part of the bitmap.
     *
     * @param      invalidatedArea The invalidated area.
     * @param [in] fb              If non-null, the fb.
     * @param      triangleXs      The triangle xs.
     * @param      triangleYs      The triangle ys.
     * @param      triangleZs      The triangle zs.
     * @param      triangleUs      The triangle us.
     * @param      triangleVs      The triangle vs.
     */
    void drawTriangle(const Rect& invalidatedArea, uint16_t* fb, const float* triangleXs, const float* triangleYs, const float* triangleZs, const float* triangleUs, const float* triangleVs) const;

    /**
     * Looks up the appropriate render variant based on the bitmap format and scaling
     * algorithm.
     *
     * Looks up the appropriate render variant based on the bitmap format and scaling
     * algorithm.
     *
     * @return A RenderingVariant.
     */
    RenderingVariant lookupRenderVariant() const;
    /// @endcond
};

} // namespace touchgfx

#endif // SCALABLEIMAGE_HPP
