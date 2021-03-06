#include <iomanip>
#include "sdl_benchmark.hpp"

sdl_benchmark::
sdl_benchmark(const std::string &about_text, bool allow_fbo):
  sdl_demo(about_text, true),
  m_avoid_allow_fbo(),
  m_common_options("Common Benchmark Options", *this),
  m_num_frames(100, "num_frame", "Number of frames to render", *this),
  m_render_to_fbo(no_fbo,
                  enumerated_string_type<enum render_to_fbo_t>()
                  .add_entry("no_fbo", no_fbo, "render directly to window")
                  .add_entry("blit_fbo", blit_fbo, "render to fbo then blit to screen")
                  .add_entry("no_blit_fbo", no_blit_fbo, "render to fbo only, no blit to screen"),
                  "render_to_fbo",
                  "Specifies to render to FBO and if so whether or not to blit the FBO to the framebuffer",
                  (allow_fbo) ? *this : m_avoid_allow_fbo),
  m_read_pixel(false, "read_pixel", "if true read the bottom right pixel just before swap_buffers()", *this),
  m_fbo_width(0, "fbo_width",
              "width of FBO to which to render (value of 0 means match window), only has effect if render_to_fbo is not no_fbo",
              (allow_fbo) ? *this : m_avoid_allow_fbo),
  m_fbo_height(0, "fbo_height",
               "height of FBO to which to render (value of 0 means match window), only has effect if render_to_fbo is not no_fbo",
               (allow_fbo) ? *this : m_avoid_allow_fbo),
  m_dry_run(false, "dry_run", "If true, do not execute any GL commands", *this),
  m_swap_buffer_extra(0, "swap_buffer_end",
                      "The number of extra times to call swap_buffers() after the last frame",
                      *this),
  m_print_each_time(false, "print_ms_each_frame",
                    "If true, print the number of ms between each frame", *this),

  m_benchmark_label("Benchmark Options", *this),

  m_fbo(0),
  m_color(0),
  m_depth_stencil(0),
  m_frame(0),
  m_to_create(0)
{
  // benchmarks do not handle events.
  sdl_demo::m_handle_events = false;
}

sdl_benchmark::
~sdl_benchmark()
{
  unbind_and_delete_fbo();
}

void
sdl_benchmark::
init_gl(int w, int h)
{
  m_screen_size=fastuidraw::ivec2(w,h);

  if (m_render_to_fbo.value()!=no_fbo)
    {
      create_and_bind_fbo();
      w = m_fbo_width.value();
      h = m_fbo_height.value();
      fastuidraw_glViewport(0, 0, w, h);
    }

  m_time.restart();
}

void
sdl_benchmark::
unbind_and_delete_fbo(void)
{
  if (m_fbo != 0)
    {
      fastuidraw_glBindFramebuffer(GL_FRAMEBUFFER, 0);
      fastuidraw_glDeleteFramebuffers(1, &m_fbo);
      fastuidraw_glDeleteTextures(1, &m_color);
      fastuidraw_glDeleteTextures(1, &m_depth_stencil);
    }
}

void
sdl_benchmark::
create_and_bind_fbo(void)
{
  if (m_fbo_width.value()==0 || m_fbo_height.value()==0)
    {
      m_fbo_width.value()=m_screen_size.x();
      m_fbo_height.value()=m_screen_size.y();
    }

  fastuidraw_glGenFramebuffers(1, &m_fbo);
  FASTUIDRAWassert(m_fbo!=0);
  fastuidraw_glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

  fastuidraw_glGenTextures(1, &m_color);
  FASTUIDRAWassert(m_color!=0);
  fastuidraw_glBindTexture(GL_TEXTURE_2D, m_color);
  fastuidraw_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  fastuidraw_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  fastuidraw_glTexImage2D(GL_TEXTURE_2D,
                          0,
                          GL_RGBA8,
                          m_fbo_width.value(), m_fbo_height.value(), 0,
                          GL_RGBA,
                          GL_UNSIGNED_BYTE,
                          nullptr);


  fastuidraw_glGenTextures(1, &m_depth_stencil);
  FASTUIDRAWassert(m_depth_stencil!=0);
  fastuidraw_glBindTexture(GL_TEXTURE_2D, m_depth_stencil);
  fastuidraw_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  fastuidraw_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  fastuidraw_glTexImage2D(GL_TEXTURE_2D,
                          0,
                          GL_DEPTH24_STENCIL8,
                          m_fbo_width.value(), m_fbo_height.value(), 0,
                          GL_DEPTH_STENCIL,
                          GL_UNSIGNED_INT_24_8,
                          nullptr);

  fastuidraw_glBindTexture(GL_TEXTURE_2D, 0);

  fastuidraw_glFramebufferTexture2D(GL_FRAMEBUFFER,
                                    GL_COLOR_ATTACHMENT0,
                                    GL_TEXTURE_2D,
                                    m_color,
                                    0);

  fastuidraw_glFramebufferTexture2D(GL_FRAMEBUFFER,
                                    GL_DEPTH_ATTACHMENT,
                                    GL_TEXTURE_2D,
                                    m_depth_stencil,
                                    0);

  fastuidraw_glFramebufferTexture2D(GL_FRAMEBUFFER,
                                    GL_STENCIL_ATTACHMENT,
                                    GL_TEXTURE_2D,
                                    m_depth_stencil,
                                    0);
}

void
sdl_benchmark::
draw_fbo_contents(void)
{
  if (m_render_to_fbo.value()==blit_fbo)
    {
      fastuidraw_glBindFramebuffer(GL_READ_FRAMEBUFFER, m_fbo);
      fastuidraw_glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
      fastuidraw_glBlitFramebuffer(0, 0, //srcX0, srcY0
                                   m_fbo_width.value(), m_fbo_height.value(), //srcX1, srcY1
                                   0, 0,
                                   m_screen_size.x(), m_screen_size.y(),
                                   GL_COLOR_BUFFER_BIT,
                                   GL_NEAREST);
      fastuidraw_glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);
    }

  if (m_read_pixel.value())
    {
      GLubyte color[4];
      fastuidraw_glReadPixels(0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, color);
      //std::cout << "\nframe " << std::setw(5) << m_frame << ":[pixel color=" << fastuidraw::ivec4(color[0], color[1], color[2], color[3]) << "]";
    }
}

void
sdl_benchmark::
draw_frame(void)
{
  if (m_print_each_time.value())
    {
      uint32_t tt;
      tt=m_last_frame_time.restart();
      if (m_frame!=0)
        {
          std::cout << "\nframe " << std::setw(5) << m_frame-1 << ": "
                    << std::setw(4) << tt << " ms" << std::flush;
        }
    }

  if (m_frame >= m_num_frames.value())
    {
      uint32_t tt;

      swap_buffers(m_swap_buffer_extra.value());
      tt = m_time.elapsed();

      switch(m_frame)
        {
        case 0:
          std::cout << "\nTook " << tt << "ms to compile shader(s) and setup state.\n";
          break;

        case 1:
          m_to_create=tt;
          //fall through
        default:
          std::cout << "\nTook " << m_to_create
                    << "ms to init GL, compile shader(s), setup state and draw first frame\n";

          if (m_frame>1)
            {
              std::cout << "After first frame, " << m_frame-1 << " frames done in "
                        << tt << " ms"
                        << "\nms/frame= "
                        << static_cast<float>(tt)/static_cast<float>(m_frame-1)
                        << "\n";
            }
        }
      end_benchmark(0);
    }
  else if (m_frame==0)
    {
      benchmark_draw_frame(0, 0);
      draw_fbo_contents();
    }
  else
    {
      if (m_frame==1)
        {
          m_to_create=m_time.restart();
        }
      benchmark_draw_frame(m_frame, m_time.elapsed());
      draw_fbo_contents();
    }
  ++m_frame;
}
