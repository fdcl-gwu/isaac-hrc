# 🔥 Fluid Dynamics with NVIDIA Omniverse™ Flow

NVIDIA Omniverse™ Flow is a powerful, real-time Eulerian fluid simulation tool for creating stunning smoke, fire, and gaseous effects within the Omniverse platform. This guide provides steps to get started using Flow in your own Omniverse projects.

## 🚀 Getting Started
If not building Isaac Sim environment from scratch, please directly go to third step to edit the fluid preset.
1. **Enable the Extension** (Please double check whether the flow extension is enable or not)  
   Open the Omniverse Extension Manager (`Window -> Extensions`) and enable `omni.flowusd`.

2. **Activate Flow in Render Settings**  
   In most cases, adding a Flow object enables this automatically. Otherwise, enable Flow under `Render Settings -> Common`.

3. **Add a Fluid Preset**  
   Go to `Window -> Simulation -> Presets`, then:
   - Drag & drop a preset into the stage
   - Or right-click to `Add to Stage` as Reference, Copy, or Emitter Only

4. **Simulate and Visualize**  
   - Press ▶️ `Play` to run the simulation  
   - Enable `Bloom` in `Post Processing` for better fire/glow visuals  
   - Use `Render Settings` to enable reflections, GI, etc.

## ⚙️ Manual Setup (Advanced)

- Add Flow primitives manually: `flowSimulate`, `flowOffscreen`, `flowRender`
- Parent emitters to `Xform` nodes for easier transformations
- Use the `Colormap` widget (under `flowOffscreen/colormap`) to tweak visual styles
- For complex scenes, assign Flow layers to separate effects cleanly

## 🔎 Monitoring & Debugging

- Open the **Flow Monitor** (`Window -> Simulation -> Monitor`) to view memory and performance
- Or enable it in the viewport via the Eye icon > `Flow Monitor`

---

## 📚 Reference

For more detailed documentation, visit the [official Omniverse Flow docs](https://docs.omniverse.nvidia.com/extensions/latest/ext_fluid-dynamics/using.html).

