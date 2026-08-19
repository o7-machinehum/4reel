# 4reel

- Camera: OV9281
- SoC: RV1103G

## Non-PRC SoC alternatives

With BGA packaging acceptable, the best documented alternative is the Microchip
SAMA7G54 SiP family:

- `SAMA7G54D1G`: 128 MB integrated DDR3L
- `SAMA7G54D2G`: 256 MB integrated DDR3L
- `SAMA7G54D4G`: 512 MB integrated DDR3L
- 1 GHz Arm Cortex-A7 with NEON and FPU
- Two-lane MIPI CSI-2, up to 1.5 Gbit/s per lane
- Linux or bare-metal/RTOS support
- 427-ball, 18 x 21 mm, 0.8 mm TFBGA, intended to be routable on four layers

Microchip provides public SiP and processor datasheets, register documentation,
IBIS models, a detailed hardware-design guide, mainline-oriented Linux support,
evaluation-board material, and CSI examples for IMX219 and OV5647.

Documentation:

- [SAMA7G54D1G product page](https://www.microchip.com/en-us/product/SAMA7G54D1G)
- [SAMA7G5 SiP datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MPU32/ProductDocuments/DataSheets/SAMA7G5-SIP-Series-Data-Sheet-DS50003577.pdf)
- [SAMA7G54 hardware-design guide](https://www.microchip.com/content/dam/mchp/documents/MPU32/ApplicationNotes/ApplicationNotes/SAMA7G54-Hardware-Design-Considerations-DS00004598.pdf)
- [Microchip camera development resources](https://developerhelp.microchip.com/xwiki/bin/view/software-tools/frameworks/vision/development-kits/)

Limitations compared with the RV1103:

- No hardware H.264/H.265 encoder
- No NPU
- Only two CSI lanes
- Image Sensor Controller for capture and basic processing rather than the
  RV1103's complete tuned ISP pipeline

If four CSI lanes matter more than CPU performance, consider the Microchip
SAM9X75 SiP family:

- `SAM9X75D1G`: 128 MB integrated DDR3L
- `SAM9X75D2G`: 256 MB integrated DDR3L
- Four-lane MIPI CSI-2
- 800 MHz Arm926
- Linux support
- 243-ball, 16 x 16 mm, 0.8 mm BGA
- No hardware video codec or NPU

Documentation:

- [SAM9X75 SiP datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MPU32/ProductDocuments/DataSheets/SAM9X75-SIP-Series-Data-Sheet-DS60001827.pdf)
- [SAM9X75 Curiosity board guide](https://ww1.microchip.com/downloads/aemDocuments/documents/MPU32/ProductDocuments/UserGuides/SAM9X75-Curiosity-User-Guide-DS60001859.pdf)

Recommendation:

- Use `SAMA7G54D2G` for general CSI capture and processing.
- Use `SAM9X75D2G` if the camera requires four lanes.
- If hardware H.265 encoding or an NPU is mandatory, neither is a complete
  RV1103 replacement; well-documented non-PRC alternatives generally require
  external DDR.
