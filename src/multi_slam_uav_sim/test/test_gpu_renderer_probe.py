import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts" / "gpu_renderer_probe.py"
)
SPEC = importlib.util.spec_from_file_location("gpu_renderer_probe", MODULE_PATH)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


class GpuRendererProbeTest(unittest.TestCase):
    def test_wsl_d3d12_adapter_is_hardware(self):
        accelerated, reason = PROBE.classify_renderer(
            "D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)", True, True
        )
        self.assertTrue(accelerated)
        self.assertEqual(reason, "wsl_d3d12_hardware_renderer")

    def test_all_known_software_renderers_are_rejected(self):
        for renderer in (
            "llvmpipe (LLVM 15.0.7, 256 bits)",
            "kms_swrast",
            "softpipe",
            "Software Rasterizer",
            "lavapipe (LLVM 17.0.0)",
            "Microsoft Basic Render Driver",
        ):
            with self.subTest(renderer=renderer):
                accelerated, reason = PROBE.classify_renderer(
                    renderer, True, True
                )
                self.assertFalse(accelerated)
                self.assertEqual(reason, "software_renderer")

    def test_wsl_requires_dxg_and_d3d12(self):
        self.assertFalse(
            PROBE.classify_renderer("D3D12 (NVIDIA)", True, False)[0]
        )
        self.assertFalse(
            PROBE.classify_renderer("NVIDIA GeForce RTX 4070", True, True)[0]
        )

    def test_native_nonsoftware_renderer_is_accepted(self):
        accelerated, reason = PROBE.classify_renderer(
            "NVIDIA GeForce RTX 4070/PCIe/SSE2", False, False
        )
        self.assertTrue(accelerated)
        self.assertEqual(reason, "native_nonsoftware_renderer")


if __name__ == "__main__":
    unittest.main()
