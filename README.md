# Simple VLM ROS2 Node

A ROS2 node that analyzes camera images using Alibaba DashScope's Vision Language Models (VLM). Subscribe to camera topics and get AI-powered image descriptions and analysis.

## Features

- 🖼️ Subscribe to ROS2 image topics
- 🤖 Analyze images using Qwen VLM models (qwen3-vl-plus, etc.)
- ⚙️ Two processing modes: Timer-based or Sequential (continuous)
- 📝 Configurable via YAML config file
- 🔌 Easy integration with existing ROS2 systems

## Prerequisites

- ROS2 (Humble or later)
- Python 3.10+
- DashScope API Key ([Get API Key](https://help.aliyun.com/zh/model-studio/get-api-key))

## Installation

### 1. Install Python Dependencies

```bash
pip install openai pyyaml opencv-python
```

### 2. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select simple_vlm
source install/setup.bash
```

## Configuration

Edit `config.yaml` with your settings:

```yaml
# DashScope API Configuration
api_key: "sk-your-api-key-here"
base_url: "https://dashscope.aliyuncs.com/compatible-mode/v1"

# ROS2 Node Configuration
image_topic: "/camera/camera/color/image_raw"
model_name: "qwen3-vl-plus"
prompt: "图中描绘的是什么景象?"

# Processing mode
sequential_mode: false  # false = timer mode, true = sequential (continuous)
process_interval: 2.0   # Seconds between processing (timer mode only)
```

### Available Regions

- **Beijing (Default)**: `https://dashscope.aliyuncs.com/compatible-mode/v1`
- **Virginia (US)**: `https://dashscope-us.aliyuncs.com/compatible-mode/v1`
- **Singapore**: `https://dashscope-intl.aliyuncs.com/compatible-mode/v1`

## Usage

### Running the Node

**Important**: If you have proxy settings, you must disable them first:

```bash
cd ~/ros2_ws
source install/setup.bash

# Disable proxy (required!)
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY

# Run with config file
ros2 run simple_vlm simple_vlm_node --ros-args -p config_file:=src/simple_vlm/config.yaml
```

### Alternative: Using Environment Variable

Instead of a config file, you can use an environment variable:

```bash
export DASHSCOPE_API_KEY="sk-your-key-here"
ros2 run simple_vlm simple_vlm_node
```

### Processing Modes

#### Timer Mode (Default)

Processes images at a regular interval:

```yaml
sequential_mode: false
process_interval: 2.0  # Process every 2 seconds
```

The node will automatically grab the latest image every 2 seconds and send it to the VLM. If still processing a previous image, it will skip that cycle.

#### Sequential Mode

Processes images continuously, as fast as possible:

```yaml
sequential_mode: true
process_interval: 2.0  # Ignored in sequential mode
```

The node will process images continuously with no delay between completions. Each new processing starts immediately after the previous one finishes, maximizing throughput.

## Topics

### Subscribed Topics

- `image_topic` (sensor_msgs/Image): Camera images to analyze
  - Default: `/camera/camera/color/image_raw`

### Published Topics

- `/vlm_result` (std_msgs/String): VLM analysis results

## Parameters

All parameters can be set via config file or command line:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_file` | string | "" | Path to YAML config file |
| `image_topic` | string | `/camera/camera/color/image_raw` | Input image topic |
| `model_name` | string | `qwen3-vl-plus` | DashScope model to use |
| `prompt` | string | `图中描绘的是什么景象?` | Question to ask about images |
| `sequential_mode` | bool | false | Use sequential (continuous) processing mode |
| `process_interval` | double | 2.0 | Processing interval in seconds (timer mode only) |

### Example: Override Parameters

```bash
ros2 run simple_vlm simple_vlm_node --ros-args \
  -p config_file:=src/simple_vlm/config.yaml \
  -p prompt:="What objects are in this image?" \
  -p process_interval:=5.0
```

## Troubleshooting

### Proxy Error

**Error**: `ValueError: Unknown scheme for proxy URL URL('socks://...')`

**Solution**: Unset proxy environment variables before running:

```bash
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
```

Or add to `~/.bashrc` to make it permanent.

### No Images Received

**Warning**: `No image received yet`

**Cause**: No camera is publishing to the configured topic.

**Solution**: 
1. Check if camera is running: `ros2 topic list | grep camera`
2. Verify topic name matches your camera: `ros2 topic echo /camera/camera/color/image_raw`
3. Update `image_topic` in config.yaml if needed

### API Key Not Found

**Error**: `No API key found! Please set DASHSCOPE_API_KEY or provide config_file`

**Solution**: 
- Set API key in `config.yaml`, OR
- Export environment variable: `export DASHSCOPE_API_KEY="sk-xxx"`

## Examples

### Basic Usage with RealSense Camera

```bash
# Terminal 1: Start camera
ros2 run realsense2_camera realsense2_camera_node

# Terminal 2: Run VLM node
cd ~/ros2_ws
source install/setup.bash
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
ros2 run simple_vlm simple_vlm_node --ros-args -p config_file:=src/simple_vlm/config.yaml

# Terminal 3: Monitor results
ros2 topic echo /vlm_result
```

### Sequential (Continuous) Processing

```yaml
# config.yaml
sequential_mode: true
prompt: "Describe what you see in detail"
```

```bash
# Node will process images continuously at maximum speed
# Each processing starts immediately after the previous one completes
```

## License

MIT

## Author

Your Name

## Links

- [DashScope API Documentation](https://help.aliyun.com/zh/model-studio/)
- [Get API Key](https://help.aliyun.com/zh/model-studio/get-api-key)
- [Qwen VLM Models](https://help.aliyun.com/zh/model-studio/models)
