# ESP32-CAM Digital Signal Processing Application

An image processing application for ESP32-CAM that demonstrates DSP techniques using the ESP-DSP library. The application captures grayscale images and applies multiple processing algorithms, saving results to an SD card with comprehensive performance logging.

## Features

- **Grayscale Image Capture**: High-quality VGA (640×480) image acquisition from OV2640 camera sensor
- **Multiple DSP Algorithms**:
  - 2D Convolution filtering (configurable kernels for edge detection, blur, sharpening)
  - Contrast stretching (linear intensity remapping)
  - Histogram equalization (automatic contrast enhancement via CDF redistribution)
- **Performance Monitoring**: Microsecond-precision timing for each processing step
- **Comprehensive Logging**: Session logs saved to SD card with timestamps and metrics
- **UUID-Based File Management**: All outputs from a session share a common UUID for easy correlation

## Hardware Requirements

- **ESP32-CAM Module** (AI-Thinker or compatible) with PSRAM
- **OV2640 Camera Sensor** (included with ESP32-CAM)
- **MicroSD Card** (FAT32 formatted, Class 10 recommended)
- **Adequate Lighting** for optimal image quality
- **USB-to-Serial Adapter** for programming and monitoring

## Output Files

Each capture session generates 5 files on the SD card:

- `{UUID}_original.pgm` - Original grayscale image
- `{UUID}_filtered.pgm` - Result after 2D convolution filtering
- `{UUID}_stretched.pgm` - Contrast-stretched image
- `{UUID}_equalized.pgm` - Histogram-equalized image
- `{UUID}_log.txt` - Processing log with timing metrics

### Example Images

#### Original Image
![Original Grayscale Image](docs/images/original.png)
*Raw grayscale capture from OV2640 sensor*

#### Convolution Filtered (Sobel Edge Detection)
![Filtered Image](docs/images/filtered.png)
*Edge detection using 3×3 Sobel X kernel*

#### Contrast Stretched
![Contrast Stretched Image](docs/images/stretched.png)
*Linear intensity remapping from [L, H] to [Bottom, Top]*

#### Histogram Equalized
![Histogram Equalized Image](docs/images/equalized.png)
*Automatic contrast enhancement via CDF redistribution*

## DSP Algorithms

### 2D Convolution Filtering

Applies a configurable N×N kernel to the image using ESP-DSP's optimized `dspi_conv_f32` function. Common kernels include:

**Edge Detection (Sobel X - Default)**
```
[-1  0  1]
[-2  0  2]
[-1  0  1]
```

**Gaussian Blur**
```
[1  2  1]
[2  4  2]
[1  2  1]
```

**Sharpening**
```
[ 0 -1  0]
[-1  5 -1]
[ 0 -1  0]
```

### Contrast Stretching

Linear intensity remapping using the formula:

```
p_adjust = Bottom + ((p - L) / (H - L)) × (Top - Bottom)
```

Where:
- `p` = original pixel intensity
- `L` = minimum intensity in image
- `H` = maximum intensity in image
- `Bottom` = target minimum (configurable, default: 50)
- `Top` = target maximum (configurable, default: 200)

### Histogram Equalization

Automatic contrast enhancement using cumulative distribution function (CDF):

```
p_eq = (CDF(p) / total_pixels) × max_level
```

This technique redistributes pixel intensities to achieve a more uniform histogram, improving overall image contrast.

## Configuration

### Kernel Customization

Edit `main/main.c` to change the convolution kernel:

```c
#define KERNEL_SIZE 3
static const float CONV_KERNEL[KERNEL_SIZE * KERNEL_SIZE] = {
    -1, 0, 1,
    -2, 0, 2,
    -1, 0, 1
};
```

### Contrast Stretch Parameters

```c
#define CONTRAST_BOTTOM 50
#define CONTRAST_TOP 200
```

### Histogram Equalization

```c
#define EQUALIZATION_MAX_LEVEL 255
```

### Camera Resolution

Edit `main/camera_driver.c`:

```c
config.frame_size = FRAMESIZE_VGA;  // Options: QVGA, VGA, SVGA, etc.
config.pixel_format = PIXFORMAT_GRAYSCALE;
```

## Building and Flashing

### Prerequisites

- ESP-IDF v5.5.1 or later
- USB-to-Serial adapter (CP2102 or CH340)
- Python 3.7+

### Build Instructions

1. Clone the repository:
```bash
git clone https://github.com/fabcontigiani/dsp-final-project.git
cd dsp-final-project
```

2. Set up ESP-IDF environment:
```bash
. $HOME/esp/esp-idf/export.sh
```

3. Configure the project (optional):
```bash
idf.py menuconfig
```

4. Build the project:
```bash
idf.py build
```

5. Flash to ESP32-CAM:
```bash
idf.py -p /dev/ttyUSB0 flash
```

6. Monitor serial output:
```bash
idf.py -p /dev/ttyUSB0 monitor
```