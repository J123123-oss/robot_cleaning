# Static Image Translated ROI Design

## Goal

Publish a fixed-size camera frame from a configured region of a static source image while applying integer pixel translations without creating blank pixels, stretched edge pixels, interpolation artifacts, or wraparound seams.

## Scope

The change is limited to `src/rtk_nav/rtk_nav/camera_publisher_node.py` and its focused tests. The existing live-camera path and ROS topic remain unchanged.

## Design

`image_path` is loaded once as the full source image. For static-image mode, `width` and `height` define the published frame size. `crop_x` and `crop_y` define the unshifted top-left corner of the source region, while `translate_x` and `translate_y` adjust that corner in integer pixels.

The frame is produced by a pure helper:

```python
extract_translated_roi(
    source, crop_x, crop_y, output_width, output_height,
    translate_x, translate_y
)
```

The effective source origin is clamped independently to:

```text
0 <= x <= source_width - output_width
0 <= y <= source_height - output_height
```

The helper returns a copied NumPy slice. It does not call affine/perspective transforms, interpolation, border filling, edge replication, reflection, or cyclic wrapping. If the source image cannot contain the requested output region, it raises `ValueError` with the source and requested dimensions.

## Parameters

The node keeps the existing `image_path`, `width`, `height`, and `fps` parameters and adds:

- `crop_x`: unshifted ROI x origin, default `0`.
- `crop_y`: unshifted ROI y origin, default `0`.
- `translate_x`: horizontal integer pixel shift, default `0`.
- `translate_y`: vertical integer pixel shift, default `0`.

For static images, each timer callback extracts the translated ROI from the unchanged source frame. The published image therefore has stable dimensions and the same encoding as before.

## Error Handling

Invalid or negative output dimensions and a source image smaller than the requested output frame are rejected before publishing. Translation values are rounded to the nearest integer because the operation is pixel slicing rather than geometric interpolation.

## Testing

Tests execute the helper directly from the source AST, matching existing repository test style. They cover positive and negative translations, both-direction boundary clamping, fixed output dimensions, and pixel provenance from the source image. A source contract test also ensures the camera publisher uses the helper and contains no affine or border-fill translation API.
