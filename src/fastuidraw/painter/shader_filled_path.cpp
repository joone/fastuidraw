/*!
 * \file shader_filled_path.cpp
 * \brief file shader_filled_path.cpp
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

#include <vector>
#include <fastuidraw/text/font.hpp>
#include <fastuidraw/text/glyph_render_data_banded_rays.hpp>
#include <fastuidraw/text/glyph_render_data_restricted_rays.hpp>
#include <fastuidraw/painter/shader_filled_path.hpp>
#include "../private/util_private.hpp"
#include "../private/bounding_box.hpp"
#include "../private/bezier_util.hpp"

namespace
{
  typedef fastuidraw::GlyphRenderDataBandedRays RenderData;
  const enum fastuidraw::glyph_type GlyphType = fastuidraw::banded_rays_glyph;

  class BuilderPrivate
  {
  public:
    RenderData m_data;
    fastuidraw::BoundingBox<float> m_bbox;
    fastuidraw::vec2 m_last_pt;
  };

  class PerFillRule
  {
  public:
    fastuidraw::vecN<fastuidraw::PainterAttribute, 4> m_attribs;
    fastuidraw::vecN<fastuidraw::PainterIndex, 6> m_indices;
    fastuidraw::vecN<fastuidraw::GlyphAttribute, RenderData::glyph_num_attributes> m_glyph_attribs;
  };

  class ShaderFilledPathPrivate
  {
  public:
    ShaderFilledPathPrivate(BuilderPrivate &B,
                            const fastuidraw::reference_counted_ptr<fastuidraw::GlyphCache> &cache);
    ~ShaderFilledPathPrivate();

    const PerFillRule&
    per_fill_rule(enum fastuidraw::PainterEnums::fill_rule_t fill_rule);

  private:
    void
    update_attribute_data(void);

    fastuidraw::reference_counted_ptr<fastuidraw::GlyphCache> m_cache;
    fastuidraw::BoundingBox<float> m_bbox;
    fastuidraw::GlyphCache::AllocationHandle m_allocation;
    std::vector<fastuidraw::generic_data> m_gpu_data;
    RenderData::query_info m_query_data;

    fastuidraw::vecN<PerFillRule, fastuidraw::PainterEnums::fill_rule_data_count> m_per_fill_rule;
    unsigned int m_number_times_atlas_cleared;
  };
}

////////////////////////////////////
// ShaderFilledPathPrivate methods
ShaderFilledPathPrivate::
ShaderFilledPathPrivate(BuilderPrivate &B,
                        const fastuidraw::reference_counted_ptr<fastuidraw::GlyphCache> &cache):
  m_cache(cache),
  m_bbox(B.m_bbox),
  m_number_times_atlas_cleared(0)
{
  using namespace fastuidraw;

  /* the fill rule actually does not matter, since ShaderFilledPath
   * constructs its own attribute data.
   */
  B.m_data.finalize(PainterEnums::nonzero_fill_rule, B.m_bbox.as_rect());
  B.m_data.query(&m_query_data);

  /* Because B may go out of scope, we need to save the GPU data
   * to our own private array.
   */
  m_gpu_data.resize(m_query_data.m_gpu_data.size());
  std::copy(m_query_data.m_gpu_data.begin(), m_query_data.m_gpu_data.end(), m_gpu_data.begin());
  m_query_data.m_gpu_data = make_c_array(m_gpu_data);
}

ShaderFilledPathPrivate::
~ShaderFilledPathPrivate()
{
  if (m_allocation.valid() && m_number_times_atlas_cleared == m_cache->number_times_atlas_cleared())
    {
      m_cache->deallocate_data(m_allocation);
    }
}

const PerFillRule&
ShaderFilledPathPrivate::
per_fill_rule(enum fastuidraw::PainterEnums::fill_rule_t fill_rule)
{
  using namespace fastuidraw;
  if (!m_allocation.valid() || m_number_times_atlas_cleared != m_cache->number_times_atlas_cleared())
    {
      m_number_times_atlas_cleared = m_cache->number_times_atlas_cleared();
      m_allocation = m_cache->allocate_data(make_c_array(m_gpu_data));
      update_attribute_data();
    }
  FASTUIDRAWassert(m_allocation.valid());
  return m_per_fill_rule[fill_rule];
}

void
ShaderFilledPathPrivate::
update_attribute_data(void)
{
  using namespace fastuidraw;
  for (unsigned int f = 0; f < m_per_fill_rule.size(); ++f)
    {
      m_query_data.set_glyph_attributes(&m_per_fill_rule[f].m_glyph_attribs,
                                        static_cast<enum PainterEnums::fill_rule_t>(f),
                                        m_allocation.location());
      Glyph::pack_raw(m_per_fill_rule[f].m_glyph_attribs,
                      0, m_per_fill_rule[f].m_attribs,
                      0, m_per_fill_rule[f].m_indices,
                      m_bbox.min_point(), m_bbox.max_point());
    }
}

/////////////////////////////////////////////////
// fastuidraw::ShaderFilledPath::Builder methods
fastuidraw::ShaderFilledPath::Builder::
Builder(void)
{
  m_d = FASTUIDRAWnew BuilderPrivate();
}

fastuidraw::ShaderFilledPath::Builder::
~Builder()
{
  BuilderPrivate *d;
  d = static_cast<BuilderPrivate*>(m_d);
  FASTUIDRAWdelete(d);
}

void
fastuidraw::ShaderFilledPath::Builder::
move_to(vec2 pt)
{
  BuilderPrivate *d;
  d = static_cast<BuilderPrivate*>(m_d);
  d->m_data.move_to(pt);
  d->m_last_pt = pt;
  d->m_bbox.union_point(pt);
}

void
fastuidraw::ShaderFilledPath::Builder::
line_to(vec2 pt)
{
  BuilderPrivate *d;
  d = static_cast<BuilderPrivate*>(m_d);
  d->m_data.line_to(pt);
  d->m_last_pt = pt;
  d->m_bbox.union_point(pt);
}

void
fastuidraw::ShaderFilledPath::Builder::
quadratic_to(vec2 ct, vec2 pt)
{
  BuilderPrivate *d;
  d = static_cast<BuilderPrivate*>(m_d);
  d->m_data.quadratic_to(ct, pt);
  d->m_last_pt = pt;
  d->m_bbox.union_point(pt);
  d->m_bbox.union_point(ct);
}

void
fastuidraw::ShaderFilledPath::Builder::
cubic_to(vec2 ct0, vec2 ct1, vec2 pt)
{
  BuilderPrivate *d;
  d = static_cast<BuilderPrivate*>(m_d);

  /* break the cubic up into pieces each
   * of which we approximate by a quadratic.
   *
   * TODO: something adaptive instead of
   * just breaking into 4 equal sized pieces
   * in the domain.
   */
  typedef vecN<vec2, 4> cubic;
  typedef vecN<vec2, 3> quadratic;

  cubic start(d->m_last_pt, ct0, ct1, pt);
  vecN<cubic, 2> s, s0, s1;
  quadratic q00, q01, q10, q11;

  s = detail::split_cubicT<float>(start);
  s0 = detail::split_cubicT<float>(s[0]);
  s1 = detail::split_cubicT<float>(s[1]);

  q00 = detail::quadratic_from_cubicT<float>(s0[0]);
  q01 = detail::quadratic_from_cubicT<float>(s0[1]);
  q10 = detail::quadratic_from_cubicT<float>(s1[0]);
  q11 = detail::quadratic_from_cubicT<float>(s1[1]);

  quadratic_to(q00[1], q00[2]);
  quadratic_to(q01[1], q01[2]);
  quadratic_to(q10[1], q10[2]);
  quadratic_to(q11[1], q11[2]);
}

////////////////////////////////////////
// fastuidraw::ShaderFilledPath methods
fastuidraw::ShaderFilledPath::
ShaderFilledPath(const Builder &B,
                 const reference_counted_ptr<GlyphCache> &glyph_cache)
{
  BuilderPrivate *bd;
  bd = static_cast<BuilderPrivate*>(B.m_d);

  m_d = FASTUIDRAWnew ShaderFilledPathPrivate(*bd, glyph_cache);
}

fastuidraw::ShaderFilledPath::
~ShaderFilledPath()
{
  ShaderFilledPathPrivate *d;
  d = static_cast<ShaderFilledPathPrivate*>(m_d);
  FASTUIDRAWdelete(d);
}

void
fastuidraw::ShaderFilledPath::
render_data(enum PainterEnums::fill_rule_t fill_rule,
            c_array<const PainterAttribute> *out_attribs,
            c_array<const PainterIndex> *out_indices) const
{
  ShaderFilledPathPrivate *d;
  d = static_cast<ShaderFilledPathPrivate*>(m_d);

  const PerFillRule &F(d->per_fill_rule(fill_rule));
  *out_attribs = F.m_attribs;
  *out_indices = F.m_indices;
}

enum fastuidraw::glyph_type
fastuidraw::ShaderFilledPath::
render_type(void) const
{
  return GlyphType;
}
