#!/bin/bash
# Optional maintainer tool: regenerates the checked-in PNG-based logo
# assets from the master SVG sources under doc/_static/logo/.
#
# This is NOT part of the documentation build. The build only ever reads
# the PNG files this script writes (they're committed to the repo like
# any other doc asset), so nothing in the build depends on rsvg-convert
# or any other SVG rasterizer being installed. Run this by hand after
# editing one of the master SVGs, and commit the results.
#
# Uses the "*-outlined" SVG variants (pdal-logo-outlined.svg,
# pdal-logo-horizontal-outlined.svg), which have the "pdal" wordmark
# converted to plain vector paths -- no font required to render them
# correctly, here or anywhere else (a browser, GitHub's SVG preview,
# etc).
#
# Also renders "-dark" variants: the same artwork with a white halo
# behind it, so the logo's dark elements (the wordmark, the circle
# outlines) stay visible when placed on a dark background instead of
# disappearing into it. The master SVGs carry a 12px buffer around the
# tight artwork bbox in their viewBox specifically so this halo has
# room to render without being clipped at the canvas edge.
set -e -u

DOC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGO_DIR="${DOC_DIR}/_static/logo"

if ! command -v rsvg-convert >/dev/null 2>&1; then
    echo "generate_logo_assets.sh: rsvg-convert not found (it's optional, not" >&2
    echo "a build dependency) -- install it (e.g. 'conda install librsvg' or" >&2
    echo "your OS's 'librsvg2-bin'/'librsvg' package) to regenerate the PNG" >&2
    echo "logo assets. Nothing to do." >&2
    exit 0
fi

render() {
    local svg="$1" out="$2" width="$3"
    rsvg-convert -w "${width}" -b white -o "${out}" "${svg}"
    echo "wrote ${out}"
}

# Renders a white-halo ("dark background safe") variant of an SVG. The
# halo thickness tracks the SVG's own viewBox width, so it automatically
# scales with the render resolution and stays inside the viewBox's 12px
# buffer (8px solid + 2px feather, leaving 2px of margin) instead of
# needing separate tuning per asset.
render_dark() {
    local svg="$1" out="$2" width="$3"
    local tmp
    tmp="$(mktemp --suffix=.png)"
    rsvg-convert -w "${width}" -b none -o "${tmp}" "${svg}"
    python3 - "${svg}" "${tmp}" "${out}" "${width}" <<'PY'
import re
import sys

from PIL import Image, ImageFilter

svg_path, src_png, out_png, width = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])

svg_text = open(svg_path, encoding="utf-8").read()
m = re.search(
    r'viewBox="[-\d.eE]+\s+[-\d.eE]+\s+([\d.eE]+)\s+[\d.eE]+"', svg_text
)
if not m:
    sys.exit(f"{svg_path}: no viewBox found")
vb_width = float(m.group(1))
scale = width / vb_width

HALO_UNITS = 8.0  # solid halo thickness, in the SVG's own user units
BLUR_UNITS = 2.0  # soft feather on the halo edge, in the same units
halo_px = max(1, round(HALO_UNITS * scale))
blur_px = max(0.5, BLUR_UNITS * scale)

img = Image.open(src_png).convert("RGBA")
mask = img.getchannel("A")

# Dilate the alpha mask by halo_px: each 3x3 max-filter pass grows the
# opaque region by ~1px, so halo_px passes gives a halo_px-thick halo.
for _ in range(halo_px):
    mask = mask.filter(ImageFilter.MaxFilter(3))
mask = mask.filter(ImageFilter.GaussianBlur(blur_px))

halo = Image.new("RGBA", img.size, (255, 255, 255, 0))
halo.putalpha(mask)

Image.alpha_composite(halo, img).save(out_png)
print("wrote", out_png)
PY
    rm -f "${tmp}"
}

render "${LOGO_DIR}/pdal-logo-outlined.svg"            "${DOC_DIR}/_static/pdal-logo.png"            1200
render "${LOGO_DIR}/pdal-logo-horizontal-outlined.svg" "${DOC_DIR}/_static/pdal-logo-horizontal.png" 1600
render_dark "${LOGO_DIR}/pdal-logo-outlined.svg"            "${DOC_DIR}/_static/pdal-logo-dark.png"            1200
render_dark "${LOGO_DIR}/pdal-logo-horizontal-outlined.svg" "${DOC_DIR}/_static/pdal-logo-horizontal-dark.png" 1600
rsvg-convert -w 256 -h 256 -b none -o "${LOGO_DIR}/favicon-256.png" "${LOGO_DIR}/pdal-favicon.svg"
python3 - "${LOGO_DIR}/favicon-256.png" "${LOGO_DIR}/favicon.ico" <<'PY'
import sys
from PIL import Image
src, dst = sys.argv[1], sys.argv[2]
im = Image.open(src).convert("RGBA")
im.save(dst, sizes=[(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
print("wrote", dst)
PY
rm -f "${LOGO_DIR}/favicon-256.png"
