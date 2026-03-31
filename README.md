# xmodelview

A standalone XModel viewer and gallery generator for **Call of Duty 1 (v1.1 - v1.5)**.

## Background

When the CoD2 modtools were released, they included an extensive XModels Library for previewing assets. Call of Duty 1 never received such a dedicated tool, making it difficult for modders and archivists to quickly browse the hundreds of models tucked away in `.pk3` files. 

`xmodelview` fills this gap by providing a lightweight, high-performance viewer that reads directly from the game's archives. It can also batch-render the entire library into a searchable web gallery.

## Features

- **Direct PK3 Reading:** No need to extract models; it reads directly from `pak0.pk3` through `pakb.pk3`.
- **Shader Support:** Parses `.shader` scripts to correctly map materials to textures (DDS, TGA, JPG).
- **Interactive Mode:** Orbit, zoom, and inspect individual models.
- **Batch Mode:** Automatically renders every model in the game to a `shots/` directory and generates a searchable `index.html` gallery.
- **Texture Validation:** A `--check-textures` mode to identify missing or undecodable assets.

## Building

Requires `SDL2`, `GLEW`, `libjpeg`, and `zlib`.

```bash
cmake -B build
cmake --build build
```

## Usage

### Interactive Viewer
```bash
./build/xmodelview xmodel/body_airborne_us_light
```
*   **Mouse Drag:** Orbit camera
*   **Scroll:** Zoom
*   **N / Right Arrow:** Next model in archive
*   **S:** Save manual screenshot
*   **Q / Esc:** Quit

### Generate Gallery (Batch Mode)
```bash
./build/xmodelview --batch --quality=90
```
This will iterate through all detected models, saving screenshots to `./shots/` and creating a searchable web interface.

### Options
- `--basepath=<path>`: Manually set the CoD1 installation directory.
- `--outdir=<path>`: Change where screenshots are saved (default: `./shots`).
- `--width=N / --height=N`: Set render resolution.
- `--check-textures`: Run a diagnostic pass to find missing textures.
