Fast UI Draw
============

Fast UI Draw is a library that provides a higher performance Canvas interface.
It is designed so that it always draws using a GPU.

In contrast to many common implementations of Canvas drawing, Fast UI Draw
has that changes in clipping are very cheap and optimized for GPU's. In
addition, Fast UI Draw has, with the GL backend, very few pipeline states.
Indeed an API trace of an application using Fast UI Draw will see only a
handful of draw calls per frame, even under high Canvas state trashing,
and high clip state changes. Indeed, for the GL backend, only one Canvas
state change invokes a pipeline state change: changing the Porter-Duff blend
mode.

In addition, Fast UI Draw gives an application the ability to make their
own shaders for custom drawing.

Documentation
=============
  Fast UI Draw uses doxygen for documentation, build and view documentation by:
  - make docs
  - xdg-open docs/html/index.html

The documentation is available online [here](https://intel.github.io/fastuidraw/docs/html/index.html).

GL requirements
=====================
  The GL backend requires GL version 3.3. For optimal rendering quality, either
  GL_ARB_shader_image_load_store or GL_EXT_shader_framebuffer_fetch is recommended.
  The extension GL_ARB_shader_image_load_store is core in GL 4.2. For optimal
  performance with optimal rendering quality, GL_EXT_shader_framebuffer_fetch or
  GL_ARB_shader_image_load_store with one of GL_INTEL_fragment_shader_ordering,
  GL_ARB_fragment_shader_interlock or GL_NV_fragment_shader_interlock is strongly
  recommended. Lastly, to support the non-seperarable blending operations,
  GL_EXT_shader_framebuffer_fetch or GL_ARB_shader_image_load_store with one of
  GL_INTEL_fragment_shader_ordering, GL_ARB_fragment_shader_interlock or
  GL_NV_fragment_shader_interlock is required. The PorterDuff composition modes
  do not require any extensions though.

  The GLES backend requires GLES version 3.0. If the GLES version is 3.0 or 3.1,
  it is strongly recommended that one of the extension GL_OES_texture_buffer or
  GL_EXT_texture_buffer is present. For GLES 3.0, 3.1 and 3.2, it is strongly
  recommended for performance to have GL_APPLE_clip_distance or GL_EXT_clip_cull_distance.
  For optimal rendering quality, one of GLES 3.1 or GL_EXT_shader_framebuffer_fetch
  is recommended. For optimal performance with optimal rendering quality,
  GL_EXT_shader_framebuffer_fetch or GLES 3.1 with GL_NV_fragment_shader_interlock
  is strongly recommended. Lastly, to support the non-seperarable blending operations,
  GL_EXT_shader_framebuffer_fetch or GLES 3.1 with GL_NV_fragment_shader_interlock
  is required. The PorterDuff composition modes do not require any extensions though,
  but the extension GL_EXT_blend_func_extended will improve performance.

  Intel GPU's starting in IvyBridge have the extensions support for optimal rendering
  with optimal performance in both GL and GLES for recent enough versions of Mesa.
  For MS-Windows, the Intel drivers for Intel GPU's also have support for optimal
  rendering with optimal performance.

Building requirements
=====================
 - GNU Make
 - g++ (clang should be fine too)
 - freetype
 - flex
 - perl
 - up to date GL (and GLES) headers
   - You need
      - for GL, from https://www.opengl.org/registry/: GL/glcorearb.h
      - for GLES, from https://www.khronos.org/registry/gles/: GLES2/gl2.h, GLES2/gl2ext.h, GLES3/gl3.h, GLES3/gl31.h, GLES3/gl32.h, KHR/khrplatform.h
   - The expected place of those headers is set by setting the
     environmental variable GL_INCLUDEPATH; if the value is not set,
     the build system will guess a value. The name of the header
     files is controlled by the environmental variables
     GL_RAW_HEADER_FILES for GL and GLES_RAW_HEADER_FILES for
     GLES. If a value is not set, reasonable defaults are used. 
 - SDL 2.0 and SDL Image 2.0 (demos only)
 - doxygen (for documentation)

Building
========
  "make targets" to see all build targets and the list of environmental
  variables that control what is built and how. On MS-Windows, the helper
  library NEGL is NOT built by default and on other platforms it is.

Installing
==========
  Doing "make INSTALL_LOCATION=/path/to/install/to install"
  will install fastuidraw to the named path. Demos are NOT
  installed! The default value of INSTALL_LOCATION is
  /usr/local, change as one sees fit. Placement is as follows:
   - INSTALL_LOCATION/lib: libraries
   - INSTALL_LOCATION/lib/pkgconfig: pkg-config files
   - INSTALL_LOCATION/include: header files
   - INSTALL_LOCATION/bin: fastuidraw-config script (and .dll's for MinGW)
   - INSTALL_LOCATION/share/doc/fastuidraw: documentation

Using project
=============
  After installation, the script, fastuidraw-config, is available
  and copied to INSTALL_LOCATION/bin. Use the script to get
  linker and compile flags for building an application. Alternatively,
  one can also use pkg-config.

Notes
=====
  - FastUIDraw has the main meat in libFastUIDraw, there are two
    variants the release and debug version whose pkg-config module
    names are fastuidraw-release and fastuidraw-debug.
  - The GL backend of FastUIDraw is libFastUIDrawGL, there are two
    variants the release and debug version whose pkg-config module
    names are fastuidrawGL-release and fastuidrawGL-debug.
  - The GLES backend of FastUIDraw is libFastUIDrawGLES, there are two
    variants the release and debug version whose pkg-config module
    names are fastuidrawGLES-release and fastuidrawGLES-debug.
  - The debug and release versions of the libraries should not be mixed;
    if you are building for release, then use the release versions and
    the release flags. If you are building for debug use the debug
    libraries and the debug flags. One can get the flag values by
    using either pkg-config or the script fastuidraw-config.
  - All demos when given -help as command line display all options
  - The demos require that the libraries are in the library path

Successfully builds under
=========================
 - Linux
 - MinGW64 and MinGW32 of MSYS2 (https://msys2.github.io/)