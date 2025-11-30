# Synthetic Data Generation

## Learning Objectives

By the end of this chapter, you will:
- Understand why synthetic data is crucial for training AI models in robotics
- Use Isaac Replicator to generate thousands of labeled images in minutes
- Configure randomization strategies (camera position, lighting, object placement)
- Export datasets in COCO format for machine learning pipelines
- Measure the efficiency gains compared to manual data labeling

## Prerequisites

- Completed Chapter 1: Isaac Sim Bridge (ROS 2 integration working)
- Python 3.9+ with PyTorch/TensorFlow installed
- RTX 4070 Ti GPU or cloud equivalent
- Isaac Sim 2024.1+ with Isaac Replicator support

## Concept: Data is Fuel

In traditional computer vision, labeling datasets is a bottleneck. A human manually annotates thousands of images:
- **Manual labeling**: ~1,000 images per person per week = **4 months for 16,000 images**
- **Manual cost**: $15,000–$50,000 in hiring and time

With **Isaac Replicator**, we flip the equation:
- **Synthetic generation**: 10,000 images in **1 hour**
- **Automatic labels**: Bounding boxes, segmentation masks, 6D poses—all free
- **Cost**: $0 (local) or &lt;$5/hour (cloud)

### Why Synthetic Data Works

1. **Perfect Ground Truth**: Simulation provides exact label information
   - Camera intrinsics and extrinsics known exactly
   - Object positions, orientations, scales known to the pixel
   - No human annotation errors

2. **Infinite Variations**: Randomize lighting, camera angles, object textures
   - Different time-of-day illumination (sunrise, noon, sunset)
   - Multiple viewing angles without moving cameras physically
   - Environmental clutter and occlusion patterns

3. **Transfer to Real**: A model trained on Isaac Sim synthetic data works on real camera footage
   - Phenomenon: "domain transfer" or "sim2real"
   - Key: Randomize heavily so the model doesn't overfit to simulation artifacts

## Metric: The 10k Images Challenge

**Scenario**: Train a robot to detect "red cubes" in its environment.

| Approach | Time | Cost | Labels | Quality |
|----------|------|------|--------|---------|
| **Manual crowdsourcing** | 4 weeks | $10k | 16,000 | Human errors, inconsistent |
| **Isaac Replicator (1 GPU)** | 1 hour | $0 | 10,000 | Perfect, reproducible |
| **Isaac Replicator (cloud)** | 30 min | $2.50 | 10,000 | Perfect, pay-as-you-go |

**The impact**: Instead of 4 weeks of waiting, you have labeled data in 1 hour. Your AI team can start training models immediately.

## Step-by-Step: Generating a Synthetic Dataset

### Step 1: Create a Simple Scene in Isaac Sim

1. Launch Isaac Sim and create a new scene:
   - **File → New** (or `Ctrl+N`)
   - Add a ground plane: **Right-click World → Create → Ground Plane**

2. Add a **target object** (e.g., red cube):
   ```
   Right-click World → Create → Cube
   - Set **Size** to (0.1, 0.1, 0.1) meters (10 cm cube)
   - Set **Color** to RGB(255, 0, 0) → red
   - Name it "RedCube"
   ```

3. Add a **camera** pointing down at the cube:
   ```
   Right-click World → Create → Camera
   - Position: (0.5, 0.5, 1.0)
   - Rotation: (0, 0, 0) facing forward
   - Name it "SensorCamera"
   ```

4. Add **lighting** (critical for realism):
   ```
   Right-click World → Create → Light → Distant Light
   - Intensity: 1.0
   - Angle: 45° (position in the sky)
   ```

### Step 2: Write a Python Script Using Isaac Replicator

Isaac Replicator is a **graph-based simulation framework** in Isaac Sim. Create this script:

**File**: `docs/03-robot-brain/code-examples/red_cube_generator.py`

```python
#!/usr/bin/env python3
"""
red_cube_generator.py - Generate synthetic dataset of red cubes using Isaac Replicator
"""
import os
import json
import numpy as np
from pathlib import Path

# Isaac Sim imports
import omni.replicator.core as rep
from omni.replicator.core import ReplicatorException

def create_synthetic_dataset(
    output_dir: str = "dataset",
    num_images: int = 1000,
    image_width: int = 1920,
    image_height: int = 1080,
):
    """
    Generate synthetic dataset with Isaac Replicator.

    Args:
        output_dir: Directory to save images and annotations
        num_images: Number of images to generate
        image_width: Image width in pixels
        image_height: Image height in pixels
    """

    # Create output directories
    output_path = Path(output_dir)
    images_dir = output_path / "images"
    annotations_dir = output_path / "annotations"
    images_dir.mkdir(parents=True, exist_ok=True)
    annotations_dir.mkdir(parents=True, exist_ok=True)

    print(f"🎬 Generating {num_images} synthetic images...")
    print(f"📁 Output directory: {output_path.absolute()}")

    # Define the scene graph for randomization
    with rep.new_layer():
        # 1. CAMERA randomization
        camera = rep.create.camera(
            position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 2.0)),
            rotation=rep.distribution.uniform((-30, -180, 0), (30, 180, 0)),
            focal_length=rep.distribution.choice([24, 35, 50])
        )
        render_product = rep.create.render_product(camera, (image_width, image_height))

        # 2. OBJECT randomization
        cube = rep.create.cube(
            size=rep.distribution.normal(0.1, 0.02),  # Size: 0.1m ± 0.02m
            position=rep.distribution.uniform((-0.5, -0.5, 0), (0.5, 0.5, 0.5)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        )

        # Set cube material to red with some variation
        def randomize_color(prim):
            """Randomize cube color to be red-ish"""
            color = rep.distribution.uniform((0.8, 0, 0), (1.0, 0.2, 0.2))
            return color

        with rep.trigger_on_frame():
            rep.modify.material_property(
                cube,
                "diffuse",
                randomize_color
            )

        # 3. LIGHTING randomization
        light = rep.create.light(
            light_type="distant",
            intensity=rep.distribution.uniform(0.5, 2.0),
            position=rep.distribution.uniform((-2, -2, 2), (2, 2, 4)),
        )

        # 4. GROUND randomization (optional: change ground texture)
        ground = rep.create.ground_plane(size=(5, 5))

        # Define data writers (output format: COCO)
        writer = rep.WriterRegistry.get("COCOWriter")
        writer.initialize(
            output_dir=str(annotations_dir),
            image_output_format="png",
            rgb_output_format="png",
        )
        writer.attach(render_product)

        # Run the simulation
        print(f"⏱️  Starting {num_images} frame simulation...")
        for frame_num in range(num_images):
            # Print progress every 100 frames
            if frame_num % 100 == 0:
                print(f"  Frame {frame_num}/{num_images} (", end="")
                print(f"{100*frame_num/num_images:.1f}%)")

            # Step the simulation
            rep.orchestrator.step()

        # Finalize dataset
        writer.finalize()
        print(f"✅ Dataset generation complete!")
        print(f"📸 Images saved to: {images_dir}")
        print(f"📋 Annotations saved to: {annotations_dir}")

        # Verify COCO annotations
        coco_file = annotations_dir / "instances_default.json"
        if coco_file.exists():
            with open(coco_file, "r") as f:
                coco_data = json.load(f)
            print(f"\n📊 Dataset Statistics:")
            print(f"   - Total images: {len(coco_data['images'])}")
            print(f"   - Total annotations: {len(coco_data['annotations'])}")
            print(f"   - Categories: {[c['name'] for c in coco_data['categories']]}")
            return True
        else:
            print(f"⚠️  Warning: COCO annotations file not found")
            return False

def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="Generate synthetic dataset with Isaac Replicator")
    parser.add_argument("--output", default="dataset", help="Output directory")
    parser.add_argument("--num-images", type=int, default=1000, help="Number of images to generate")
    parser.add_argument("--width", type=int, default=1920, help="Image width")
    parser.add_argument("--height", type=int, default=1080, help="Image height")

    args = parser.parse_args()

    success = create_synthetic_dataset(
        output_dir=args.output,
        num_images=args.num_images,
        image_width=args.width,
        image_height=args.height,
    )

    return 0 if success else 1

if __name__ == "__main__":
    main()
```

### Step 3: Configure the Replicator Graph

Before running the script, configure the **Omnigraph** in Isaac Sim:

1. **Windows → Omnigraph → Action Graph Editor**
2. Create nodes for:
   - **On Tick**: Trigger each frame
   - **SetVariable**: Track frame counter
   - **IsaacReadCamera**: Capture RGB and instance segmentation
   - **COCOWriter**: Write annotations in COCO format

Alternative (simpler): Use the script above—it handles all graph configuration via Python API.

### Step 4: Run the Generator

Execute the script inside Isaac Sim:

```bash
# Option 1: Standalone script (inside Isaac Sim Python environment)
cd /path/to/isaac-sim
./python.sh /path/to/red_cube_generator.py --num-images 1000

# Option 2: From terminal (if Isaac Sim supports headless mode)
python3 red_cube_generator.py --num-images 1000 --output dataset
```

**Expected output**:
```
🎬 Generating 1000 synthetic images...
📁 Output directory: /home/user/dataset
⏱️  Starting 1000 frame simulation...
  Frame 0/1000 (0.0%)
  Frame 100/1000 (10.0%)
  Frame 200/1000 (20.0%)
  ...
  Frame 900/1000 (90.0%)
✅ Dataset generation complete!
📸 Images saved to: /home/user/dataset/images
📋 Annotations saved to: /home/user/dataset/annotations

📊 Dataset Statistics:
   - Total images: 1000
   - Total annotations: 1005
   - Categories: ['RedCube']
```

**Performance on RTX 4070 Ti**: ~1,000 images in **3–5 minutes** (including disk I/O).

## Dataset Structure and COCO Format

After generation, your dataset looks like:

```
dataset/
├── images/
│   ├── 0000.png      (1920×1080 RGB image, frame 0)
│   ├── 0001.png      (frame 1)
│   ├── ...
│   └── 0999.png      (frame 999)
└── annotations/
    └── instances_default.json   (COCO format)
```

### COCO JSON Structure

The `instances_default.json` file:

```json
{
  "info": {
    "description": "COCO dataset generated by Isaac Replicator",
    "version": "1.0"
  },
  "images": [
    {
      "id": 0,
      "file_name": "0000.png",
      "height": 1080,
      "width": 1920
    },
    {
      "id": 1,
      "file_name": "0001.png",
      "height": 1080,
      "width": 1920
    }
  ],
  "annotations": [
    {
      "id": 0,
      "image_id": 0,
      "category_id": 1,
      "bbox": [100, 150, 50, 50],  # [x, y, width, height]
      "area": 2500,
      "iscrowd": 0
    }
  ],
  "categories": [
    {
      "id": 1,
      "name": "RedCube",
      "supercategory": "object"
    }
  ]
}
```

## Code Snippet: Dataset Validation

Verify your dataset integrity:

**File**: `docs/03-robot-brain/code-examples/validate_dataset.py`

```python
#!/usr/bin/env python3
"""
validate_dataset.py - Validate synthetic dataset
"""
import json
import cv2
from pathlib import Path

def validate_coco_dataset(dataset_dir: str):
    """Check dataset consistency and image validity"""

    dataset_path = Path(dataset_dir)
    images_dir = dataset_path / "images"
    coco_file = dataset_path / "annotations" / "instances_default.json"

    print(f"🔍 Validating dataset at: {dataset_path}")

    # Load COCO annotations
    with open(coco_file, "r") as f:
        coco = json.load(f)

    images = {img['id']: img for img in coco['images']}
    annotations = coco['annotations']

    # Check 1: Image file existence
    print(f"✓ Checking {len(images)} image files...")
    missing = 0
    for img_id, img_meta in images.items():
        img_path = images_dir / img_meta['file_name']
        if not img_path.exists():
            print(f"  ⚠️  Missing: {img_meta['file_name']}")
            missing += 1

    if missing == 0:
        print(f"  ✅ All {len(images)} images found")
    else:
        print(f"  ❌ {missing}/{len(images)} images missing")

    # Check 2: Annotation validity
    print(f"✓ Checking {len(annotations)} annotations...")
    valid = 0
    for ann in annotations:
        bbox = ann['bbox']
        if bbox[2] > 0 and bbox[3] > 0:  # width and height must be positive
            valid += 1

    print(f"  ✅ {valid}/{len(annotations)} valid bounding boxes")

    # Check 3: Sample image inspection
    print(f"✓ Sampling first image...")
    first_img_meta = images[0]
    first_img_path = images_dir / first_img_meta['file_name']
    img = cv2.imread(str(first_img_path))

    if img is not None:
        h, w, c = img.shape
        print(f"  ✅ Image shape: {w}×{h}×{c}")
        if (h, w) == (first_img_meta['height'], first_img_meta['width']):
            print(f"  ✅ Metadata matches image dimensions")
        else:
            print(f"  ❌ Metadata mismatch")
    else:
        print(f"  ❌ Failed to read image")

    return missing == 0 and valid == len(annotations)

if __name__ == "__main__":
    import sys
    dataset_dir = sys.argv[1] if len(sys.argv) > 1 else "dataset"
    validate_coco_dataset(dataset_dir)
```

## Troubleshooting

### Problem: Generator produces black/blank images

**Cause**: Camera not pointed at objects, or lighting is zero.

**Solutions**:
1. Verify camera position and rotation in Isaac Sim viewport
2. Add a `Distant Light` to the scene with intensity > 0.5
3. Move objects to camera's field of view (camera looks forward, objects should be in front)

### Problem: COCO annotations file is empty

**Cause**: COCOWriter not attached to render product or simulation didn't step forward.

**Solutions**:
1. Ensure `writer.attach(render_product)` is called
2. Check that `rep.orchestrator.step()` is being called in the loop
3. Verify write permissions to output directory

### Problem: Generator runs very slowly (&lt; 100 images/min)

**Cause**: GPU underutilized or disk bottleneck.

**Solutions**:
1. Reduce image resolution (e.g., 1280×720 instead of 1920×1080)
2. Use SSD or faster disk for output
3. Increase batch size if using cloud GPU

## Configuration Example: Randomization Parameters

Save this as `docs/03-robot-brain/configs/replicator_config.yaml`:

```yaml
# Isaac Replicator Configuration

output:
  format: "coco"              # COCO, YOLO, or custom
  num_images: 1000            # Total frames to generate
  image_width: 1920
  image_height: 1080
  output_dir: "dataset"

camera:
  position_min: [-1, -1, 0.5]
  position_max: [1, 1, 2.0]
  rotation_min: [-30, -180, 0]
  rotation_max: [30, 180, 0]
  focal_lengths: [24, 35, 50]

object:
  size_mean: 0.1              # 10 cm
  size_std: 0.02              # ±2 cm
  color_min: [0.8, 0.0, 0.0]  # RGB min (red)
  color_max: [1.0, 0.2, 0.2]  # RGB max (light red)

lighting:
  intensity_min: 0.5
  intensity_max: 2.0
  position_min: [-2, -2, 2]
  position_max: [2, 2, 4]

domain_randomization:
  enable_texture_randomization: true
  enable_material_randomization: true
  num_variations_per_scene: 10
```

## Key Takeaways

✅ **Synthetic data generation** eliminates the manual labeling bottleneck
✅ **Isaac Replicator** generates 10,000+ labeled images per hour on RTX 4070 Ti
✅ **Randomization strategies** (camera, lighting, object placement) prevent model overfitting
✅ **COCO format** integrates seamlessly with machine learning frameworks (Detectron2, YOLO)
✅ **Perfect ground truth** from simulation enables highly accurate models

## Estimated Completion Time

- **Reading**: 40–50 minutes
- **Hands-on lab** (generating 1000 images): 30 minutes
- **Validation and exploration**: 20 minutes
- **Total**: 90–100 minutes

## Hardware Requirements

| Platform | Status | Notes |
|----------|--------|-------|
| **RTX 4070 Ti** | ✅ Primary | 1000 images in 3–5 minutes |
| **Jetson Orin** | ⚠️ Limited | Reduced batch size, 10–20 min per 1000 images |
| **Cloud (Omniverse)** | ✅ Alternative | &lt;$5/hour, comparable to local RTX 4070 Ti |

---

**Next chapter**: [Visual SLAM](./03-visual-slam.md) — Use the camera data to build a 3D map of the environment in real-time.
