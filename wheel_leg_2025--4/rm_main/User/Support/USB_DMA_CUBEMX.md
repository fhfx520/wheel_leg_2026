# USB DMA and CubeMX regeneration

USB DMA is always enabled. With D-Cache enabled, MPU region 7 maps
0x24048000..0x2404BFFF as Normal non-cacheable, non-bufferable, execute-never
memory. With D-Cache disabled, the MPU setup is skipped.

## Preserved integration points

- `USB_DEVICE/Target/usbd_conf.c`, USER CODE Includes: custom memory header.
- USER CODE PV: aligned extern declaration for the generated PCD handle.
- USER CODE 0: MPU setup, scatter-symbol checks, and local HAL transmit/receive
  wrappers that issue DSB before handing a buffer to DMA. The HAL-name aliases
  are local to this translation unit and appear after the wrapper bodies.
- USER CODE USB_OTG_HS_MspInit 0: MPU setup and explicit DMA enable, before
  HAL initializes the USB core and enables its DMA.
- `USB_DEVICE/App/usbd_cdc_if.c`, USER CODE PV: aligned extern declarations for
  the generated RX/TX arrays. The actual definitions remain generator-owned;
  ArmClang applies the declarations' 32-byte alignment to the same objects.
  Do not add duplicate buffer definitions to a USER CODE section.

## Custom files and project configuration

`User/Support/usb_dma_memory.h` and `User/Log/fault_noinit.sct` own the reserved
memory layout. The scatter selects the PCD/setup, device/control, descriptors,
CDC class allocation and RX/TX arrays. The large CPU-only log queue stays in
ordinary RAM. Class allocation retains CubeMX's uint32_t alignment; cache-line
alignment is not needed for that non-cacheable storage.

Keep `ProjectManager.KeepUserCode=true` and USB DMA enabled in `rm_main.ioc`.
The Keil target must continue selecting `../User/Log/fault_noinit.sct`, with the
existing `User/Support` include directory. If regeneration resets the scatter
selection, references to `Image$$RW_USB_DMA$$Base/Limit` intentionally make the
link fail rather than silently produce cache-incoherent USB firmware.

## Validation performed

Keil compilation/linking and map inspection verify buffer placement and
alignment. After stripping existing USER CODE sections, both modified USB C
files match the original generated bodies in the desktop reference project.
An actual CubeMX regeneration has not been run against the user's project.
After regeneration, rebuild and check the map and sustained HOST_CLASS traffic.
