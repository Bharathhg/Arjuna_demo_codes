import yaml
import cv2
import numpy as np
import os


def load_map(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    yaml_dir = os.path.dirname(yaml_file)

    # Correct image path
    image_path = os.path.join(yaml_dir, data['image'])

    resolution = data['resolution']
    origin = data['origin']

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise Exception(f"❌ Failed to load image: {image_path}")

    print(f"✅ Loaded map: {image_path}")

    return img, resolution, origin


def generate_world(img, resolution, origin, output_file):
    height, width = img.shape
    threshold = 200

    world = []

    world.append("""
<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
    """)

    box_id = 0

    for y in range(height):
        for x in range(width):
            if img[y, x] < threshold:

                wx = origin[0] + x * resolution
                wy = origin[1] + (height - y) * resolution

                world.append(f"""
    <model name="wall_{box_id}">
      <static>true</static>
      <pose>{wx} {wy} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{resolution} {resolution} 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{resolution} {resolution} 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
                """)
                box_id += 1

    world.append("""
  </world>
</sdf>
    """)

    with open(output_file, 'w') as f:
        f.write("\n".join(world))

    print(f"✅ World generated: {output_file}")
    print(f"Total obstacles: {box_id}")


# 🔥 HARDCODED MAP PATH (your path)
if __name__ == "__main__":
    yaml_file = "/home/hacker/arjuna2_ws/src/arjuna/arjuna/maps/my_map.yaml"
    output_world = "/home/hacker/arjuna2_ws/src/arjuna_description/worlds/arjuna.world"

    img, resolution, origin = load_map(yaml_file)
    generate_world(img, resolution, origin, output_world)