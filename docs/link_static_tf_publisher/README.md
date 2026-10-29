# LinkStaticTFPublisher

`LinkStaticTFPublisher` publishes the fixed transform between two links of a Gazebo model. It reads their relative pose once when the plugin loads.

Load `liblink_static_tf_publisher.so` inside the model. Implementation: [plugins/link_static_tf_publisher.cpp](../../plugins/link_static_tf_publisher.cpp).

## Interfaces

The inputs are the two model links and their simulated poses at startup. The output is a static transform on `/tf_static` (`tf2_msgs/msg/TFMessage`). Use this plugin for links whose relative pose remains fixed.

## Configuration

| SDF field | Meaning |
| --- | --- |
| `robotNamespace` | Prefix for the published parent and child frame IDs. |
| `parentLink` | Required existing parent link. |
| `childLink` | Required existing child link. |

Use link names containing `_link`. The plugin removes that substring when forming frame IDs: with `robotNamespace=uav1`, `base_link` becomes `uav1/base`.

For example, inside a model containing `base_link` and `camera_link`:

```xml
<plugin name="camera_tf" filename="liblink_static_tf_publisher.so">
  <robotNamespace>uav1</robotNamespace>
  <parentLink>base_link</parentLink>
  <childLink>camera_link</childLink>
</plugin>
```

This publishes `uav1/base` to `uav1/camera` using the links' mounting pose.

[Package overview](../../README.md)
