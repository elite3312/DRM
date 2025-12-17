# macrosilicon drm linux

This is the Linux DRM driver for Macrosilicon MS91xx series devices taken from <http://www.macrosilicon.com/download/USBDisplay/Linux/SourceCode/MS91xx_Linux_Drm_SourceCode_V3.0.3.13.zip>.

This repo fixes a bug in the original source code that caused frame persistence artifacts during screen updates.

## Fix frame persistence artifacts during screen updates

The driver was experiencing persistence of old frame data when content
changed rapidly on screen. Root cause was a race condition in the dirty
rectangle tracking logic.

Key issues fixed:

1. image_buf was being updated immediately after change detection, but
   before the frame was transmitted over USB. If a new frame arrived
   during transmission, image_buf and desktop_buf would become out of
   sync, causing incorrect dirty region detection and stale pixels being
   sent to the hardware.

2. usb_hal_combine_rects() was updating both rectangle buffers instead
   of only the current frame's buffer, causing stale regions to persist
   across frames.


Solution: Defer image_buf updates until after successful USB transmission,
ensuring it always reflects what's actually displayed on hardware. Only
update the current frame's dirty rectangle.

## build

only tested on ubuntu 22.04 with kernel 5.15.

```sh
sudo insmod ./drm/usbdisp_drm.ko
sudo insmod ./drm/usbdisp_usb.ko
sudo systemctl restart display-manager
```