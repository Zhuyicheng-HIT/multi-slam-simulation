#!/usr/bin/env python3
"""Create an EGL context and report the OpenGL renderer used by Gazebo."""

import ctypes
import os
import platform
from pathlib import Path


EGL_OPENGL_ES_API = 0x30A0
EGL_SURFACE_TYPE = 0x3033
EGL_PBUFFER_BIT = 0x0001
EGL_RED_SIZE = 0x3024
EGL_GREEN_SIZE = 0x3023
EGL_BLUE_SIZE = 0x3022
EGL_RENDERABLE_TYPE = 0x3040
EGL_OPENGL_ES2_BIT = 0x0004
EGL_WIDTH = 0x3057
EGL_HEIGHT = 0x3056
EGL_CONTEXT_CLIENT_VERSION = 0x3098
EGL_NONE = 0x3038
EGL_VENDOR = 0x3053
EGL_VERSION = 0x3054

GL_VENDOR = 0x1F00
GL_RENDERER = 0x1F01
GL_VERSION = 0x1F02

SOFTWARE_RENDERER_MARKERS = (
    "llvmpipe",
    "softpipe",
    "swrast",
    "software rasterizer",
    "lavapipe",
    "microsoft basic render driver",
)


def is_wsl():
    return "microsoft" in platform.release().lower()


def classify_renderer(renderer, wsl, has_dxg):
    """Return (accelerated, reason) from the actual GL renderer string."""
    normalized = renderer.strip().lower()
    if not normalized:
        return False, "missing_renderer"
    if any(marker in normalized for marker in SOFTWARE_RENDERER_MARKERS):
        return False, "software_renderer"
    if wsl:
        if not has_dxg:
            return False, "wsl_dxg_missing"
        if "d3d12" not in normalized:
            return False, "wsl_renderer_is_not_d3d12"
        return True, "wsl_d3d12_hardware_renderer"
    return True, "native_nonsoftware_renderer"


def decode(value):
    return value.decode("utf-8", errors="replace") if value else "unknown"


def egl_error(egl):
    egl.eglGetError.restype = ctypes.c_uint
    return f"0x{egl.eglGetError():04x}"


def create_probe_context():
    egl = ctypes.CDLL("libEGL.so.1")
    gles = ctypes.CDLL("libGLESv2.so.2")

    egl.eglGetDisplay.argtypes = [ctypes.c_void_p]
    egl.eglGetDisplay.restype = ctypes.c_void_p
    egl.eglInitialize.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    egl.eglInitialize.restype = ctypes.c_uint
    egl.eglBindAPI.argtypes = [ctypes.c_uint]
    egl.eglBindAPI.restype = ctypes.c_uint
    egl.eglChooseConfig.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
    ]
    egl.eglChooseConfig.restype = ctypes.c_uint
    egl.eglCreatePbufferSurface.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
    ]
    egl.eglCreatePbufferSurface.restype = ctypes.c_void_p
    egl.eglCreateContext.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
    ]
    egl.eglCreateContext.restype = ctypes.c_void_p
    egl.eglMakeCurrent.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    egl.eglMakeCurrent.restype = ctypes.c_uint
    egl.eglQueryString.argtypes = [ctypes.c_void_p, ctypes.c_int]
    egl.eglQueryString.restype = ctypes.c_char_p
    egl.eglDestroyContext.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    egl.eglDestroySurface.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    egl.eglTerminate.argtypes = [ctypes.c_void_p]

    gles.glGetString.argtypes = [ctypes.c_uint]
    gles.glGetString.restype = ctypes.c_char_p

    display = egl.eglGetDisplay(ctypes.c_void_p(0))
    if not display:
        raise RuntimeError(f"eglGetDisplay failed ({egl_error(egl)})")

    major = ctypes.c_int()
    minor = ctypes.c_int()
    if not egl.eglInitialize(display, ctypes.byref(major), ctypes.byref(minor)):
        raise RuntimeError(f"eglInitialize failed ({egl_error(egl)})")

    surface = ctypes.c_void_p(0)
    context = ctypes.c_void_p(0)
    try:
        if not egl.eglBindAPI(EGL_OPENGL_ES_API):
            raise RuntimeError(f"eglBindAPI failed ({egl_error(egl)})")

        config_attributes = (ctypes.c_int * 11)(
            EGL_SURFACE_TYPE,
            EGL_PBUFFER_BIT,
            EGL_RED_SIZE,
            8,
            EGL_GREEN_SIZE,
            8,
            EGL_BLUE_SIZE,
            8,
            EGL_RENDERABLE_TYPE,
            EGL_OPENGL_ES2_BIT,
            EGL_NONE,
        )
        config = ctypes.c_void_p()
        config_count = ctypes.c_int()
        if not egl.eglChooseConfig(
            display,
            config_attributes,
            ctypes.byref(config),
            1,
            ctypes.byref(config_count),
        ) or config_count.value < 1:
            raise RuntimeError(f"eglChooseConfig failed ({egl_error(egl)})")

        surface_attributes = (ctypes.c_int * 5)(
            EGL_WIDTH,
            1,
            EGL_HEIGHT,
            1,
            EGL_NONE,
        )
        surface = egl.eglCreatePbufferSurface(
            display, config, surface_attributes
        )
        if not surface:
            raise RuntimeError(
                f"eglCreatePbufferSurface failed ({egl_error(egl)})"
            )

        context_attributes = (ctypes.c_int * 3)(
            EGL_CONTEXT_CLIENT_VERSION,
            2,
            EGL_NONE,
        )
        context = egl.eglCreateContext(
            display, config, ctypes.c_void_p(0), context_attributes
        )
        if not context:
            raise RuntimeError(f"eglCreateContext failed ({egl_error(egl)})")
        if not egl.eglMakeCurrent(display, surface, surface, context):
            raise RuntimeError(f"eglMakeCurrent failed ({egl_error(egl)})")

        return {
            "probe_backend": "egl_gles_pbuffer",
            "egl_vendor": decode(egl.eglQueryString(display, EGL_VENDOR)),
            "egl_version": decode(egl.eglQueryString(display, EGL_VERSION)),
            "gl_vendor": decode(gles.glGetString(GL_VENDOR)),
            "gl_renderer": decode(gles.glGetString(GL_RENDERER)),
            "gl_version": decode(gles.glGetString(GL_VERSION)),
        }
    finally:
        if context:
            egl.eglMakeCurrent(
                display,
                ctypes.c_void_p(0),
                ctypes.c_void_p(0),
                ctypes.c_void_p(0),
            )
            egl.eglDestroyContext(display, context)
        if surface:
            egl.eglDestroySurface(display, surface)
        egl.eglTerminate(display)


def main():
    try:
        report = create_probe_context()
    except (OSError, RuntimeError) as error:
        print("probe_backend=egl_gles_pbuffer")
        print("hardware_accelerated=no")
        print(f"hardware_reason=probe_failed:{error}")
        return 1

    accelerated, reason = classify_renderer(
        report["gl_renderer"], is_wsl(), Path("/dev/dxg").exists()
    )
    report["hardware_accelerated"] = "yes" if accelerated else "no"
    report["hardware_reason"] = reason
    report["mesa_adapter_request"] = os.environ.get(
        "MESA_D3D12_DEFAULT_ADAPTER_NAME", "auto"
    )
    for key, value in report.items():
        print(f"{key}={value}")
    return 0 if accelerated else 2


if __name__ == "__main__":
    raise SystemExit(main())
