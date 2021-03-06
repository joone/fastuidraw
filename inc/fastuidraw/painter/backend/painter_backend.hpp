/*!
 * \file painter_backend.hpp
 * \brief file painter_backend.hpp
 *
 * Copyright 2016 by Intel.
 *
 * Contact: kevin.rogovin@intel.com
 *
 * This Source Code Form is subject to the
 * terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with
 * this file, You can obtain one at
 * http://mozilla.org/MPL/2.0/.
 *
 * \author Kevin Rogovin <kevin.rogovin@intel.com>
 *
 */


#pragma once

#include <fastuidraw/util/blend_mode.hpp>
#include <fastuidraw/util/rect.hpp>
#include <fastuidraw/text/glyph_atlas.hpp>
#include <fastuidraw/image.hpp>
#include <fastuidraw/colorstop_atlas.hpp>
#include <fastuidraw/painter/backend/painter_draw.hpp>
#include <fastuidraw/painter/backend/painter_shader_registrar.hpp>


namespace fastuidraw
{
/*!\addtogroup PainterBackend
 * @{
 */

  /*!
   * \brief
   * A PainterBackend is an interface that defines the API-specific
   * elements to implement Painter:
   */
  class PainterBackend:public reference_counted<PainterBackend>::default_base
  {
  public:
    /*!
     * \brief
     * A ConfigurationBase holds how data should be set to a
     * PainterBackend
     */
    class ConfigurationBase
    {
    public:
      /*!
       * Ctor.
       */
      ConfigurationBase(void);

      /*!
       * Copy ctor.
       */
      ConfigurationBase(const ConfigurationBase &obj);

      ~ConfigurationBase();

      /*!
       * assignment operator
       */
      ConfigurationBase&
      operator=(const ConfigurationBase &obj);

      /*!
       * Swap operation
       * \param obj object with which to swap
       */
      void
      swap(ConfigurationBase &obj);

      /*!
       * Bits that are up in brush_shader_mask(void) that change
       * in PainterBrush::shader() trigger a call to
       * PainterDraw::draw_break().
       */
      uint32_t
      brush_shader_mask(void) const;

      /*!
       * Specify the value returned by brush_shader_mask(void) const,
       * default value is 0
       * \param v value
       */
      ConfigurationBase&
      brush_shader_mask(uint32_t v);

      /*!
       * Returns the PainterCompositeShader::shader_type the \ref
       * PainterBackend accepts for \ref PainterCompositeShader
       * objects.
       */
      enum PainterCompositeShader::shader_type
      composite_type(void) const;

      /*!
       * Specify the return value to composite_type() const.
       * Default value is \ref PainterCompositeShader::dual_src.
       * \param tp composite shader type
       */
      ConfigurationBase&
      composite_type(enum PainterCompositeShader::shader_type tp);

      /*!
       * If true, indicates that the PainterBackend supports
       * bindless texturing. Default value is false.
       */
      bool
      supports_bindless_texturing(void) const;

      /*!
       * Specify the return value to supports_bindless_texturing() const.
       * Default value is false.
       */
      ConfigurationBase&
      supports_bindless_texturing(bool);

    private:
      void *m_d;
    };

    /*!
     * \brief
     * PerformanceHints provides miscellaneous data about
     * an implementation of a PainterBackend.
     */
    class PerformanceHints
    {
    public:
      /*!
       * Ctor.
       */
      PerformanceHints(void);

      /*!
       * Copy ctor.
       */
      PerformanceHints(const PerformanceHints &obj);

      ~PerformanceHints();

      /*!
       * assignment operator
       */
      PerformanceHints&
      operator=(const PerformanceHints &obj);

      /*!
       * Swap operation
       * \param obj object with which to swap
       */
      void
      swap(PerformanceHints &obj);

      /*!
       * Returns true if an implementation of PainterBackend
       * clips triangles (for example by a hardware clipper
       * or geometry shading) instead of discard to implement
       * clipping as embodied by \ref PainterClipEquations.
       */
      bool
      clipping_via_hw_clip_planes(void) const;

      /*!
       * Set the value returned by
       * clipping_via_hw_clip_planes(void) const,
       * default value is true.
       */
      PerformanceHints&
      clipping_via_hw_clip_planes(bool v);

      /*!
       * Gives the maximum z-value an implementation of
       * PainterBackend support.
       */
      int
      max_z(void) const;

      /*!
       * Set the value returned by max_z(void) const,
       * default value is 2^20.
       */
      PerformanceHints&
      max_z(int);

    private:
      void *m_d;
    };

    /*!
     * Surface represents an interface to specify a buffer to
     * which a PainterBackend renders content.
     */
    class Surface:public reference_counted<Surface>::default_base
    {
    public:
      /*!
       * A Viewport specifies the sub-region within a Surface
       * to which one renders.
       */
      class Viewport
      {
      public:
        Viewport(void):
          m_origin(0, 0),
          m_dimensions(1, 1)
        {}

        /*!
         * Ctor.
         * \param x value with which to initialize x-coordinate of \ref m_origin
         * \param y value with which to initialize y-coordinate of \ref m_origin
         * \param w value with which to initialize x-coordinate of \ref m_dimensions
         * \param h value with which to initialize y-coordinate of \ref m_dimensions
         */
        Viewport(int x, int y, int w, int h):
          m_origin(x, y),
          m_dimensions(w, h)
        {}

        /*!
         * Compute pixel coordinates from normalized device coords
         * using this Viewport values. The pixel coordinates are so
         * that (0, 0) is the bottom left.
         * \param ndc normalized device coordinates
         */
        vec2
        compute_pixel_coordinates(vec2 ndc) const
        {
          ndc += vec2(1.0f); // place in range [0, 2]
          ndc *= 0.5f;       // normalize to [0, 1]
          ndc *= vec2(m_dimensions); // normalize to dimension
          ndc += vec2(m_origin); // translate to origin
          return ndc;
        }

        /*!
         * Compute normalized device coordinates from pixel
         * coordinates.
         * \param pixel pixel coordinates where (0, 0) corresponds to bottom
         *              left of the surface
         */
        vec2
        compute_normalized_device_coords(vec2 pixel) const
        {
          pixel -= vec2(m_origin); // translate from origin
          pixel /= vec2(m_dimensions); // normalize to [0, 1]
          pixel *= 2.0f; // normalize to [0, 2]
          pixel -= vec2(1.0f); // palce in range [-1, 1]
          return pixel;
        }

        /*!
         * Computes the clip-equations (in normalized device coordinates)
         * of this Viewport against a surface with the given dimensions.
         * \param surface_dims dimension of surface
         * \param[out] out_clip_equations location to which to write the
         *                                clip equations
         */
        void
        compute_clip_equations(ivec2 surface_dims,
                               vecN<vec3, 4> *out_clip_equations) const;

        /*!
         * Computes the rectangle in normalized device coordinates
         * of the intersection of a backing surface with the given
         * dimensions against this Viewport.
         * \param surface_dims dimension of surface
         * \param[out] out_rect location to which to write the
         *                      clipping rectangle in normalized
         *                      device coordinates
         */
        void
        compute_normalized_clip_rect(ivec2 surface_dims,
                                     Rect *out_rect) const;

        /*!
         * Returne the size needed by a surface to contain
         * the viewport, i.e. how many pixels the viewport
         * covers.
         */
        ivec2
        visible_dimensions(void) const
        {
          ivec2 return_value(m_dimensions);

          /* remove from the portion of the viewport that
           * is below/left of the surface
           */
          return_value.x() += t_min(0, m_origin.x());
          return_value.y() += t_min(0, m_origin.y());
          return return_value;
        }

        /*!
         * Computes the dimensions of the intersection
         * of this viewport against a surface with the
         * given resolution.
         * \param surface_dims dimension of surface
         */
        ivec2
        compute_visible_dimensions(ivec2 surface_dims) const
        {
          ivec2 return_value(visible_dimensions());

          return_value.x() = t_min(return_value.x(), surface_dims.x());
          return_value.y() = t_min(return_value.y(), surface_dims.y());
          return return_value;
        }

        /*!
         * The origin of the viewport
         */
        ivec2 m_origin;

        /*!
         * The dimensions of the viewport
         */
        ivec2 m_dimensions;
      };

      virtual
      ~Surface()
      {}

      /*!
       * Return an \ref Image whose backing is the same as
       * this PainterBackend::Surface. It is expected that
       * backing \ref Image is the same for the lifetime of
       * the PainterBackend::Surface. The caller gaurantees
       * that the same ImageAtlas object will be passed on
       * each call to image().
       * \param atlas ImageAtlas to manage the returned Image
       */
      virtual
      reference_counted_ptr<const Image>
      image(const reference_counted_ptr<ImageAtlas> &atlas) const = 0;

      /*!
       * To be implemented by a derived class to return
       * the viewport into the Surface.
       */
      virtual
      const Viewport&
      viewport(void) const = 0;

      /*!
       * To be implemented by a derived class to set
       * the viewport into the surface. The viewport
       * cannot be changed while the Surface is in
       * use by a \ref PainterBackend or \ref Painter.
       * \param vwp new viewport into the surface to use
       */
      virtual
      void
      viewport(const Viewport &vwp) = 0;

      /*!
       * To be implemented by a derived class to return
       * the clear color.
       */
      virtual
      const vec4&
      clear_color(void) const = 0;

      /*!
       * To be implemented by a derived class to set
       * the clear color.
       */
      virtual
      void
      clear_color(const vec4&) = 0;

      /*!
       * To be implemented by a derived class to return
       * the dimensions of the Surface backing store.
       */
      virtual
      ivec2
      dimensions(void) const = 0;
    };

    /*!
     * Ctor.
     * \param glyph_atlas GlyphAtlas for glyphs drawn by the PainterBackend
     * \param image_atlas ImageAtlas for images drawn by the PainterBackend
     * \param colorstop_atlas ColorStopAtlas for color stop sequences drawn by the PainterBackend
     * \param shader_registrar PainterShaderRegistrar to which shaders are registered
     * \param config ConfigurationBase for how to pack data to PainterBackend
     * \param pdefault_shaders default shaders for PainterBackend; shaders are
     *                         registered at constructor.
     */
    PainterBackend(reference_counted_ptr<GlyphAtlas> glyph_atlas,
                   reference_counted_ptr<ImageAtlas> image_atlas,
                   reference_counted_ptr<ColorStopAtlas> colorstop_atlas,
                   reference_counted_ptr<PainterShaderRegistrar> shader_registrar,
                   const ConfigurationBase &config,
                   const PainterShaderSet &pdefault_shaders);

    virtual
    ~PainterBackend();

    /*!
     * To be implemented by a derived class to return
     * the number of attributes a PainterDraw returned
     * by map_draw() is guaranteed to hold.
     */
    virtual
    unsigned int
    attribs_per_mapping(void) const = 0;

    /*!
     * To be implemented by a derived class to return
     * the number of indices a PainterDraw returned
     * by map_draw() is guaranteed to hold.
     */
    virtual
    unsigned int
    indices_per_mapping(void) const = 0;

    /*!
     * Returns a handle to the GlyphAtlas of this
     * PainterBackend. All glyphs used by this
     * PainterBackend must live on glyph_atlas().
     */
    const reference_counted_ptr<GlyphAtlas>&
    glyph_atlas(void);

    /*!
     * Returns a handle to the ImageAtlas of this
     * PainterBackend. All images used by all brushes
     * of this PainterBackend must live on image_atlas().
     */
    const reference_counted_ptr<ImageAtlas>&
    image_atlas(void);

    /*!
     * Returns a handle to the ColorStopAtlas of this
     * PainterBackend. All color stops used by all brushes
     * of this PainterBackend must live on colorstop_atlas().
     */
    const reference_counted_ptr<ColorStopAtlas>&
    colorstop_atlas(void);

    /*!
     * Returns the PainterShaderRegistrar of this PainterBackend.
     * Use this return value to add custom shaders. NOTE: shaders
     * added within a thread are not useable within that thread
     * until the next call to begin().
     */
    const reference_counted_ptr<PainterShaderRegistrar>&
    painter_shader_registrar(void);

    /*!
     * Returns the ConfigurationBase passed in the ctor.
     */
    const ConfigurationBase&
    configuration_base(void) const;

    /*!
     * To be implemented by a derived class to create another
     * PainterBackend object which uses the same atlases,
     * has the -exact- same \ref PainterShaderRegistrar and
     * is configured exactly the same way.
     */
    virtual
    reference_counted_ptr<PainterBackend>
    create_shared(void) = 0;

    /*!
     * Called just before calling PainterDraw::draw() on a sequence
     * of PainterDraw objects who have had their PainterDraw::unmap()
     * routine called. An implementation will  will clear the depth
     * (aka occlusion) buffer and optionally the color buffer in the
     * viewport of the \ref PainterBackend::Surface.
     * \param surface the \ref PainterBackend::Surface to which to
     *                render content
     * \param clear_color_buffer if true, clear the color buffer
     *                           on the viewport of the surface.
     * \param begin_new_target if true indicates that drawing is to
     *                         start on the surface (typically this
     *                         means that when this is true that the
     *                         backend will clear all auxiliary buffers
     *                         (such as the depth buffer).
     */
    virtual
    void
    on_pre_draw(const reference_counted_ptr<Surface> &surface,
                bool clear_color_buffer,
                bool begin_new_target) = 0;

    /*!
     * Called just after calling PainterDraw::draw()
     * on a sequence of PainterDraw objects.
     */
    virtual
    void
    on_post_draw(void) = 0;

    /*!
     * Called to return an action to bind an Image whose backing
     * store requires API binding.
     * \param im Image backed by a gfx API surface that in order to be used,
     *           must be bound. In patricular im's Image::type() value
     *           is Image::context_texture2d
     */
    virtual
    reference_counted_ptr<PainterDraw::Action>
    bind_image(const reference_counted_ptr<const Image> &im) = 0;

    /*!
     * To be implemented by a derived class to return a PainterDraw
     * for filling of data.
     */
    virtual
    reference_counted_ptr<PainterDraw>
    map_draw(void) = 0;

    /*!
     * To be implemented by a derived class to create a
     * Surface with its own backing that is useable by
     * both the creating \ref PainterBackend and any \ref
     * PainterBackend returned by create_shared().
     * \param dims the dimensions of the backing store of
     *             the returned Surface
     */
    virtual
    reference_counted_ptr<Surface>
    create_surface(ivec2 dims) = 0;

    /*!
     * Returns the PainterShaderSet for the backend.
     * Returned values will already be registerd by the
     * backend.
     */
    const PainterShaderSet&
    default_shaders(void);

    /*!
     * Returns the PerformanceHints for the PainterBackend,
     * may only be called after on_begin() has been called
     * atleast once. The value returned is expected to stay
     * constant once on_begin() has been called.
     */
    const PerformanceHints&
    hints(void) const;

  protected:
    /*!
     * To be accessed by a derived class in its ctor
     * to set the performance hint values for itself.
     */
    PerformanceHints&
    set_hints(void);

  private:
    void *m_d;
  };
/*! @} */

}
